/*!
 * ClockDivider
 * Copyright 2026 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#include <Arduino.h>
#include <numeric>
#include <hardware/pwm.h>
#include <hardware/irq.h>
#include <hardware/gpio.h>
#include <EEPROM.h>
#include "lib/Button.hpp"
#include "lib/RotaryEncoder.hpp"
#include "lib/RGBLEDPWMControl.hpp"
#include "lib/EEPROMConfigIO.hpp"
#include "triggerhappy_gpio.h"
#include "triggerhappy_basicconfig.h"
#include "SystemConfig.hpp"

#include "lib/pwm_wrapper.h"
#include "lib/EdgeChecker.hpp"
#include "lib/TriggerOut.hpp"
#include "TriggerOutManager.hpp"

#include "ClockDividerMultiplier.hpp"
#include "SyncLFO.hpp"
#include "PatternSequencer.hpp"
#include "Patterns.hpp"

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
    HRE = 0x0003,
    HA_HB = 0x0330,
    HM_HRE = 0x3003,
};

// 標準インターフェース
// static uint interruptSliceNum;
static RotaryEncoder enc;
static Button buttons[4];
static RGBLEDPWMControl rgbLedControl;
static RGBLEDPWMControl::MenuColor menuColor = RGBLEDPWMControl::MenuColor::GREEN;
static EEPROMConfigIO<SystemConfig> systemConfig(0);

// 機能
static int8_t trigModeIndex = 0;
// static EdgeChecker clockEdge;
// static EdgeChecker resetEdge;
static EdgeChecker dataEdge;
static volatile bool clockEdgeLatch = false;
static volatile bool resetEdgeLatch = false;
static volatile bool dataEdgeLatch = false;
static TriggerOutManager triggerOutManager;

class PatSeqImplA : public PatternSequencer
{
private:
    void onOutputHigh(uint8_t ch) override
    {
        triggerOutManager.out(ch)->set(1);
    }

    void onOutputLow(uint8_t ch) override
    {
        triggerOutManager.out(ch)->set(0);
    }
};
PatSeqImplA patSeq;

class ClockDiviImplB : public ClockDividerMultiplier
{
public:
    ClockDiviImplB(uint8_t numChannels): ClockDividerMultiplier(numChannels) {}

private:
    void onOutputHigh(uint8_t ch) override
    {
        triggerOutManager.out(ch + 3)->set(1);
    }

    void onOutputLow(uint8_t ch) override
    {
        triggerOutManager.out(ch + 3)->set(0);
    }
};
ClockDiviImplB clockDivB(3);

SyncLFO lfo;

PatternSelect patSel;


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
    trigModeIndex = constrainCyclic(trigModeIndex + delta, 0, 6);
    const RGBLEDPWMControl::MenuColor menuColors[OUT_COUNT + 1] = {
        RGBLEDPWMControl::MenuColor::GREEN,
        RGBLEDPWMControl::MenuColor::GREEN,
        RGBLEDPWMControl::MenuColor::YELLOW,
        RGBLEDPWMControl::MenuColor::YELLOW,
        RGBLEDPWMControl::MenuColor::RED,
        RGBLEDPWMControl::MenuColor::RED,
        RGBLEDPWMControl::MenuColor::CYAN};
    const int8_t menuLevels[OUT_COUNT + 1] = {
        5, 11,
        5, 11,
        5, 11,
        11};

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
    if (buttonStates == ButtonCondition::HA)
    {
        lfo.addWaveform(encValue);
    }
    if (buttonStates == ButtonCondition::HB)
    {
        clockDivB.allAddPulseMode(encValue);
    }
    if (buttonStates == ButtonCondition::HM)
    {
        if (trigModeIndex == 6)
        {
        }
        else if (trigModeIndex < 3)
        {
            patSeq.requieredAllOn(trigModeIndex, true);
        }
        else
        {
        }
    }
    if (buttonStates == ButtonCondition::HRE)
    {
        if (trigModeIndex == 6)
        {
        }
        else if (trigModeIndex < 3)
        {
            patSeq.requieredMute(trigModeIndex, true);
        }
        else
        {
        }
    }
    else if (buttonStates == ButtonCondition::HM_HRE)
    {
        patSeq.onResetRise();
    }
    else if (buttonStates == ButtonCondition::NONE)
    {
        if (trigModeIndex == 6)
        {
            lfo.addRatio(encValue);
        }
        else if (trigModeIndex < 3)
        {
            patSeq.requieredMute(trigModeIndex, false);
            patSeq.requieredAllOn(trigModeIndex, false);
            patSel.addIndex(trigModeIndex, encValue);
            patSeq.setPattern(trigModeIndex, patSel.getPattern(trigModeIndex), 64);
        }
        else
        {
            clockDivB.channels[trigModeIndex - 3].addRatio(encValue);
        }
    }
}

void process()
{
    if (resetEdgeLatch)
    {
        resetEdgeLatch = false;
        patSeq.onResetRise();
        clockDivB.onResetRise();
        lfo.onResetRise();
        clockEdgeLatch = true;
    }
    else
    {
        if (clockEdgeLatch)
        {
            clockEdgeLatch = false;
            lfo.onClockRise();
            patSeq.onClockRise();
            if (!dataEdge.isAlive())
            {
                clockDivB.onClockRise();
            }
        }

        if (dataEdgeLatch)
        {
            dataEdgeLatch = false;
            clockDivB.onClockRise();
        }
    }

    patSeq.update();
    clockDivB.update();
    lfo.update();

    pwm_set_gpio_level(OUT_CV, lfo.getValue());
    triggerOutManager.process();
}

//////////////////////////////////////////

void edgeCallback(uint gpio, uint32_t events)
{
    if (gpio == CLOCK)
    {
        if (events & GPIO_IRQ_EDGE_RISE)
        {
            // clockEdge.updateEdge(1);
            clockEdgeLatch = true;
        }
        else if (events & GPIO_IRQ_EDGE_FALL)
        {
            // clockEdge.updateEdge(0);
            clockEdgeLatch = false;
        }
    }
    else if (gpio == RESET)
    {
        if (events & GPIO_IRQ_EDGE_RISE)
        {
            // resetEdge.updateEdge(1);
            resetEdgeLatch = true;
        }
        else if (events & GPIO_IRQ_EDGE_FALL)
        {
            // resetEdge.updateEdge(0);
            resetEdgeLatch = false;
        }
    }
    else if (gpio == DATA)
    {
        if (events & GPIO_IRQ_EDGE_RISE)
        {
            dataEdge.updateEdge(1);
            dataEdgeLatch = true;
        }
        else if (events & GPIO_IRQ_EDGE_FALL)
        {
            dataEdge.updateEdge(0);
            dataEdgeLatch = false;
        }
    }
}

// void interruptPWM()
// {
//     pwm_clear_irq(interruptSliceNum);
// }

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
    buttons[2].setHoldTime(10);
    buttons[3].init(BTN_MODE);
    buttons[3].setHoldTime(10);
    // clockEdge.init(CLOCK);
    // resetEdge.init(RESET);
    dataEdge.init(DATA, 2000); // 入力チェックのみ利用
    triggerOutManager.init();

    rgbLedControl.init(20000, PWM_BIT, LED_R, LED_G, LED_B);
    rgbLedControl.setMenuColor(menuColor);

    systemConfig.initEEPROM();
    systemConfig.loadUserConfig();

    patSeq.init();
    patSeq.setPattern(0, patSel.getPattern(0), 64);
    patSeq.setPattern(1, patSel.getPattern(1), 64);
    patSeq.setPattern(2, patSel.getPattern(2), 64);
    clockDivB.init();
    clockDivB.channels[0].setRatio(ClockDividerMultiplier::RatioIndex::CLK);
    clockDivB.channels[1].setRatio(ClockDividerMultiplier::RatioIndex::DIV32);
    clockDivB.channels[2].setRatio(ClockDividerMultiplier::RatioIndex::DIV64);
    clockDivB.allSetPulseMode(ClockDividerMultiplier::PulseMode::TRIGGER);

    lfo.init(PWM_BIT);
    lfo.setWaveform(SyncLFO::Waveform::TRIANGLE);
    lfo.setRatio(SyncLFO::RatioIndex::DIV64);

    initPWM(OUT_CV, PWM_RESO);
    // initPWMIntr(PWM_INTR_PIN, interruptPWM, &interruptSliceNum, SAMPLE_FREQ, INTR_PWM_RESO, CPU_CLOCK);

    pinMode(CLOCK, INPUT);
    pinMode(RESET, INPUT);
    gpio_set_irq_enabled(CLOCK, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(RESET, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(DATA, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_callback(edgeCallback);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

void loop()
{
    int8_t encValue = enc.getDirection();

    process();

    rgbLedControl.process();
    tight_loop_contents();
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
