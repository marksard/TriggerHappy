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
static RotaryEncoder enc;
static Button buttons[4];
static RGBLEDPWMControl rgbLedControl;
static RGBLEDPWMControl::MenuColor menuColor = RGBLEDPWMControl::MenuColor::GREEN;
static EEPROMConfigIO<SystemConfig> systemConfig(0);

// 機能
static int8_t trigModeIndex = 0;
static EdgeChecker clockEdge;
static EdgeChecker resetEdge;
static EdgeChecker dataEdge;
static TriggerOutManager triggerOutManager;

class ClockDiviImplA : public ClockDividerMultiplier
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
ClockDiviImplA clockDivA;
class ClockDiviImplB : public ClockDividerMultiplier
{
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
ClockDiviImplB clockDivB;

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
        RGBLEDPWMControl::MenuColor::RED};
    const int8_t menuLevels[OUT_COUNT] = {
        5, 11,
        5, 11,
        5, 11};

    rgbLedControl.setMenuColor(menuColors[trigModeIndex]);
    rgbLedControl.setMenuColorLevel(menuLevels[trigModeIndex]);
}

//////////////////////////////////////////

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
    clockEdge.init(CLOCK);
    resetEdge.init(RESET);
    dataEdge.init(DATA, 2000);
    triggerOutManager.init();

    rgbLedControl.init(20000, PWM_BIT, LED_R, LED_G, LED_B);
    rgbLedControl.setMenuColor(menuColor);

    systemConfig.initEEPROM();
    systemConfig.loadUserConfig();


    clockDivA.init();
    clockDivA.channels[0].setRatio(ClockDividerMultiplier::RatioIndex::MUL1);
    clockDivA.channels[1].setRatio(ClockDividerMultiplier::RatioIndex::DIV2);
    clockDivA.channels[2].setRatio(ClockDividerMultiplier::RatioIndex::DIV4);
    clockDivB.init();
    clockDivB.channels[0].setRatio(ClockDividerMultiplier::RatioIndex::DIV3);
    clockDivB.channels[1].setRatio(ClockDividerMultiplier::RatioIndex::DIV5);
    clockDivB.channels[2].setRatio(ClockDividerMultiplier::RatioIndex::DIV7);
}

void loop()
{
    int8_t encValue = enc.getDirection();

    if (resetEdge.isEdgeHigh())
    {
        clockDivA.onResetRise();
        clockDivB.onResetRise();
    }
    if (clockEdge.isEdgeHigh())
    {
        clockDivA.onClockRise();
        if (! dataEdge.isAlive())
        {
            clockDivB.onClockRise();
        }
    }
    clockDivA.update();

    if (dataEdge.isEdgeHigh())
    {
        clockDivB.onClockRise();
    }
    clockDivB.update();

    triggerOutManager.process();

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
            clockDivA.allAddPulseMode(encValue);
            clockDivB.allAddPulseMode(encValue);
        }
        else if (buttonStates == ButtonCondition::UMODE)
        {
        }
        else if (buttonStates == ButtonCondition::NONE)
        {
            if (trigModeIndex < 3)
            {
                clockDivA.channels[trigModeIndex].addRatio(encValue);
            }
            else
            {
                clockDivB.channels[trigModeIndex - 3].addRatio(encValue);
            }
        }
    }

    rgbLedControl.update();
    tight_loop_contents();
    sleep_ms(10);
}
