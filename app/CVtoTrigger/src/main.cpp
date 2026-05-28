/*!
 * CV to Trigger
 * Copyright 2026 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

/*
# CV to Trigger

CV to Triggerは以下の機能を備えています。  

1. CV入力をクロック入力のタイミングで6bit量子化でビットごとにトリガー出力を行います。
1. CVクオンタイザー+S&Hも備えており、上記とは別のクロック入力によりCV入力をクオンタイズ、ホールド出力します。

## 機能概要

- 入力
  - `CLOCK` 6bitサンプリングクロック入力
  - `RESET` クオンタイザー用S&H入力
  - `DATA` CV入力
- 出力
  - `OUT1~6` 6bit出力
  - `CV` クオンタイザー出力

- 操作方法
  - `B` 次のOUT chのトリガー設定を選択（エンコーダーLEDが各ch出力確認LEDと同色に変化します）
  - `A` 前のOUT chのトリガー設定を選択（エンコーダーLEDが各ch出力確認LEDと同色に変化します）
  - `MODE` 選択されたOUT chのゲート出力・トリガー出力を切り替え（出力確認LEDで確認）
  - `ロータリーエンコーダー` 全OUT chトリガー出力の長さを選択します（7段階）
  - `A長押し`+`ロータリーエンコーダー` クオンタイザー出力のオクターブ範囲を設定します（5段階）
  - `B長押し`+`ロータリーエンコーダー` クオンタイザー出力のスケールを設定します（メジャー/ナチュラルマイナー）
  */

#include <Arduino.h>
#include <numeric>
#include <hardware/pwm.h>
#include <hardware/irq.h>
#include <hardware/gpio.h>
#include <EEPROM.h>
#include "lib/Button.hpp"
#include "lib/SmoothAnalogRead.hpp"
#include "lib/RotaryEncoder.hpp"
#include "lib/ADCErrorCorrection.hpp"
#include "lib/RGBLEDPWMControl.hpp"
#include "lib/EEPROMConfigIO.hpp"
#include "triggerhappy_gpio.h"
#include "triggerhappy_basicconfig.h"
#include "SystemConfig.hpp"

#include "lib/pwm_wrapper.h"
#include "lib/RandomFast.hpp"
#include "lib/EdgeChecker.hpp"
#include "lib/TriggerOut.hpp"
#include "lib/Quantizer.hpp"
#include "lib/Euclidean.hpp"
#include "lib/MiniOsc.hpp"
#include "TriggerOutManager.hpp"

enum ButtonCondition
{
    // 各ボタンの押下状態と各ボタンの組み合わせ
    // Button State: 0:None 1:Button down 2:Button up 3:Holding 4:Holded
    // U: button up
    // D: button down
    // H: holging
    // L: holded (leaved)
    // 0xMABR (Mode, A, B, RE(RotaryEncoder) button)
    NONE = 0x0000,
    UA = 0x0200,
    UB = 0x0020,
    UMODE = 0x2000,
    URE = 0x0002,
    HA = 0x0300,
    HB = 0x0030,
    HM = 0x3000,
    HA_HB = 0x0330,
    HM_URE = 0x3002,
};

// 標準インターフェース
static uint interruptSliceNum;
static RotaryEncoder enc;
static Button buttons[4];
static SmoothAnalogRead dataIn;
static RGBLEDPWMControl rgbLedControl;
static RGBLEDPWMControl::MenuColor menuColor = RGBLEDPWMControl::MenuColor::GREEN;
static ADCErrorCorrection adcErrorCorrection;

// gpio割り込み
static volatile bool clockEdgeLatch = false;
static volatile bool dataEdgeLatch = false;
static volatile bool resetEdgeLatch = false;
static EdgeChecker clockEdge;

// UIほか

// 機能
static TriggerOutManager triggerOutManager;
static Quantizer quantizer(PWM_RESO);
uint8_t r2rOctMax = 2;
uint8_t r2rScale = 1;
const int8_t r2rScaleIndex[2] = {0, 5}; // maj, min
static bool triggerModes[OUT_COUNT] = {1, 1, 1, 1, 1, 1};
static int8_t trigModeIndex = 0;
static EEPROMConfigIO<SystemConfig> systemConfig(0);

//////////////////////////////////////////

template <typename vs = int8_t>
vs constrainCyclic(vs value, vs min, vs max)
{
    if (value > max)
        return min;
    if (value < min)
        return max;
    return value;
}

//////////////////////////////////////////

void addTriggerModeCh(int8_t delta)
{
    trigModeIndex = constrainCyclic(trigModeIndex + delta, 0, 5);
    const RGBLEDPWMControl::MenuColor menuColors[OUT_COUNT] = {
        RGBLEDPWMControl::MenuColor::GREEN,
        RGBLEDPWMControl::MenuColor::GREEN,
        RGBLEDPWMControl::MenuColor::YELLOW,
        RGBLEDPWMControl::MenuColor::YELLOW,
        RGBLEDPWMControl::MenuColor::RED,
        RGBLEDPWMControl::MenuColor::RED
    };
    const int8_t menuLevels[OUT_COUNT] = {
        5, 11,
        5, 11,
        5, 11
    };

    rgbLedControl.setMenuColor(menuColors[trigModeIndex]);
    rgbLedControl.setMenuColorLevel(menuLevels[trigModeIndex]);
}

void operation(uint16_t buttonStates, int8_t encValue)
{
    if (buttonStates == ButtonCondition::UA)
    {
        addTriggerModeCh(-1);
    }
    else if (buttonStates == ButtonCondition::UB)
    {
        addTriggerModeCh(1);
    }
    else if (buttonStates == ButtonCondition::UMODE)
    {
        triggerModes[trigModeIndex] = triggerModes[trigModeIndex] ? 0 : 1; // 0:gate 1:trig
    }
    else if (buttonStates == ButtonCondition::URE)
    {
    }
    else if (buttonStates == ButtonCondition::HM_URE)
    {
    }
    else if (buttonStates == ButtonCondition::HA)
    {
        r2rOctMax = constrain(r2rOctMax + (encValue), 1, 5);
        rgbLedControl.setRainbowLevel(r2rOctMax, 1, 5);
    }
    else if (buttonStates == ButtonCondition::HB)
    {
        r2rScale = constrain(r2rScale + (encValue), 0, 1);
        rgbLedControl.setRainbowLevel(r2rScale, 0, 1);
    }
    else if (buttonStates == ButtonCondition::NONE)
    {
        triggerOutManager.addDurationMode(encValue);
    }
}

void process(int16_t dataInValue)
{
    int16_t downSampleDataInValue = dataInValue >> 6;
    Serial.println(downSampleDataInValue & 0x02, 2);
    if (clockEdgeLatch)
    {
        clockEdgeLatch = false;
        for (int i = 0; i < OUT_COUNT; ++i)
        {
            int duration = clockEdge.getDurationMills();
            duration = map(triggerOutManager.getDuration(), 0, 100, 0, duration);
            triggerOutManager.out(i)->setDuration(duration);
            int8_t trig = downSampleDataInValue & (1 << i);
            trig = triggerOutManager.out(i)->getTriggerGate(
                trig > 0 ? 1 : 0,
                triggerModes[i]);
                // triggerOutManager.isGateMode() ? 0 : 1);
            triggerOutManager.out(i)->set(trig);
        }
    }
    else
    {
        for (int i = 0; i < OUT_COUNT; ++i)
        {
            triggerOutManager.out(i)->update(0);
        }
    }

    if (resetEdgeLatch)
    {
        resetEdgeLatch = false;
        uint16_t semi = map(dataInValue, 0, ADC_RESO, 0, (7 * r2rOctMax));
        uint16_t r2rCV = quantizer.Quantize(semi);
        r2rCV = constrain(r2rCV - PWMCVDCOutputErrorLUT[semi], 0, PWM_RESO - 1);
        pwm_set_gpio_level(OUT_CV, r2rCV);
    }

    triggerOutManager.process();
}

//////////////////////////////////////////

void edgeCallback(uint gpio, uint32_t events)
{
    if (gpio == CLOCK)
    {
        if (events & GPIO_IRQ_EDGE_RISE)
        {
            clockEdge.updateEdge(1);
            clockEdgeLatch = true;
        }
        else if (events & GPIO_IRQ_EDGE_FALL)
        {
            clockEdge.updateEdge(0);
            clockEdgeLatch = false;
        }
    }
    else if (gpio == RESET)
    {
        if (events & GPIO_IRQ_EDGE_RISE)
        {
            resetEdgeLatch = true;
        }
        else if (events & GPIO_IRQ_EDGE_FALL)
        {
            resetEdgeLatch = false;
        }
    }
}

void interruptPWM()
{
    pwm_clear_irq(interruptSliceNum);
}

void setup()
{
    set_sys_clock_hz(CPU_CLOCK, true);
    pinMode(23, OUTPUT);
    gpio_put(23, HIGH);

    enc.init(EC1B, EC1A, true);
    buttons[0].init(BTN_A);
    buttons[0].setHoldTime(350);
    buttons[1].init(BTN_B);
    buttons[1].setHoldTime(350);
    buttons[2].init(BTN_RE, false, false, true);
    buttons[2].setHoldTime(500);
    buttons[3].init(BTN_MODE);
    buttons[3].setHoldTime(350);
    dataIn.init(DATA);
    clockEdge.init(CLOCK); // clockエッジ期間計測のみで利用
    triggerOutManager.init();

    rgbLedControl.init(20000, PWM_BIT, LED_R, LED_G, LED_B);
    rgbLedControl.setMenuColor(menuColor);

    systemConfig.initEEPROM();
    systemConfig.loadUserConfig();

    adcErrorCorrection.init(systemConfig.Config.vRef, systemConfig.Config.noiseFloor);

    quantizer.setScale(r2rScaleIndex[r2rScale]);

    initPWM(OUT_CV, PWM_RESO);

    initPWMIntr(PWM_INTR_PIN, interruptPWM, &interruptSliceNum, SAMPLE_FREQ, INTR_PWM_RESO, CPU_CLOCK);

    gpio_init(CLOCK);
    gpio_init(RESET);
    gpio_set_irq_enabled(CLOCK, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(RESET, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_callback(edgeCallback);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

void loop()
{
    int8_t encValue = enc.getDirection();
    int16_t dataInValue = dataIn.analogReadDirectFast();

    process(dataInValue);

    rgbLedControl.process();
    tight_loop_contents();
    sleep_us(50);
}

void setup1()
{
}

void loop1()
{
    int8_t encValue = enc.getValue();
    uint8_t btnA = buttons[0].getState();
    uint8_t btnB = buttons[1].getState();
    uint8_t btnRE = buttons[2].getState();
    uint8_t btnMode = buttons[3].getState();
    // ButtonCondition用にまとめる
    uint16_t buttonStates = (btnMode << 12) + (btnA << 8) + (btnB << 4) + btnRE;

    operation(buttonStates, encValue);

    rgbLedControl.update();
    tight_loop_contents();
    sleep_ms(10);
}
