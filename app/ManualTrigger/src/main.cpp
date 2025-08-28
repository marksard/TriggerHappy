/*!
 * Trigger Happy
 * Copyright 2025 marksard
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
static SmoothAnalogRead clockIn;
static SmoothAnalogRead resetIn;
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
static MiniOsc internalClock;
static volatile bool useInternalClock = false;
static bool settingMode = false;
static EEPROMConfigIO<SystemConfig> systemConfig(0);

// test
#define STEP_COUNT 16
static int8_t stepTriggers[OUT_COUNT][STEP_COUNT] = {
    {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0},
    {1, 0, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1},
    {1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 1, 1, 0},
    {0, 0, 1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1},
    {1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 0, 1},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};
static int8_t stepTriggerCurrent[OUT_COUNT] = {0};
static int8_t currentStep = 0;
static bool cvInputMode = false;
static int16_t cvSequence[STEP_COUNT] = {
    7, 3, 7, 7, 1, 0, 7, 14,
    0, 7, 7, 14, 0, 7, 14, 4};
static int8_t cvSequenceCurrent = 0;
static int8_t stepTriggerWork[STEP_COUNT] = {0};
static int8_t cvSequenceWork[STEP_COUNT] = 
    {7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7};

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

void process(int16_t dataInValue)
{
    if (clockEdgeLatch)
    {
        clockEdgeLatch = false;
        for (int i = 0; i < OUT_COUNT; ++i)
        {
            int8_t onset = stepTriggers[i][stepTriggerCurrent[i]];
            triggerOutManager.out(i)->setDuration(clockEdge.getDurationMills() >> 2);
            onset = triggerOutManager.out(i)->getTriggerGate(onset > 0 ? 1 : 0, onset == 2 ? 0 : 1);
            triggerOutManager.out(i)->set(onset);

            stepTriggerCurrent[i]++;
            onset = stepTriggers[i][stepTriggerCurrent[i]];
            if (stepTriggerCurrent[i] > STEP_COUNT - 1 ||
                onset == -1)
            {
                stepTriggerCurrent[i] = 0;
            }
        }

        int16_t value = cvSequence[cvSequenceCurrent];
        uint8_t oct = value / 7;
        uint8_t semi = quantizer.Scales[5][value % 7];
        uint8_t octSemi = ((oct * 12) + semi);
        uint16_t cvValue = octSemi * quantizer.VoltPerTone;
        cvValue = constrain(cvValue - PWMCVDCOutputErrorLUT[octSemi], 0, PWM_RESO - 1);
        pwm_set_gpio_level(OUT_CV, cvValue);
        cvSequenceCurrent++;
        if (cvSequenceCurrent > STEP_COUNT - 1 ||
            cvSequence[cvSequenceCurrent] == -1)
        {
            cvSequenceCurrent = 0;
        }

        currentStep = constrainCyclic(currentStep + 1, 0, STEP_COUNT - 1);
    }
    else
    {
        for (int i = 0; i < OUT_COUNT; ++i)
        {
            triggerOutManager.out(i)->update(0);
        }
    }

    triggerOutManager.process();
}

void confirmStepTrigger(int16_t &inputStep, bool &isRec, int16_t selectCh)
{
    inputStep = 0;
    isRec = false;
    for (int i = 0; i < STEP_COUNT; ++i)
    {
        stepTriggers[selectCh][i] = stepTriggerWork[i];
        stepTriggerWork[i] = 0;
    }
}

void receptOperationTriggers(uint16_t buttonStates, int8_t encValue)
{
    static bool isRec = false;
    static int16_t selectCh = 0;
    static int16_t inputStep = 0;

    if (isRec == false)
    {
        if (buttonStates == ButtonCondition::UA)
        {
        }
        else if (buttonStates == ButtonCondition::UB)
        {
        }
        else if (buttonStates == ButtonCondition::UMODE)
        {
            inputStep = 0;
            isRec = true;
        }
        else if (buttonStates == ButtonCondition::URE)
        {
            currentStep = 0;
            cvSequenceCurrent = 0;
            for (int i = 0; i < OUT_COUNT; ++i)
            {
                stepTriggerCurrent[i] = 0;
            }
        }
        else if (buttonStates == ButtonCondition::HM_URE)
        {
            cvInputMode = true;
        }
        else if (buttonStates == ButtonCondition::HA_HB)
        {
        }
        else if (buttonStates == ButtonCondition::HA)
        {
        }
        else if (buttonStates == ButtonCondition::HB)
        {
        }
        else if (buttonStates == ButtonCondition::HM)
        {
        }
        else if (buttonStates == ButtonCondition::NONE)
        {
            selectCh = constrain(selectCh + encValue, 0, OUT_COUNT - 1);
        }

        const RGBLEDPWMControl::MenuColor cols[6] = {
            RGBLEDPWMControl::MenuColor::GREEN,
            RGBLEDPWMControl::MenuColor::GREEN,
            RGBLEDPWMControl::MenuColor::YELLOW,
            RGBLEDPWMControl::MenuColor::YELLOW,
            RGBLEDPWMControl::MenuColor::RED,
            RGBLEDPWMControl::MenuColor::RED};
        const int8_t levels[6] = {4, 8, 4, 8, 4, 8};

        rgbLedControl.setBLevel(currentStep == 0 || currentStep == (STEP_COUNT - 1) ? 11 : 0);
        rgbLedControl.setMenuColor(cols[selectCh]);
        rgbLedControl.setMenuColorLevel(levels[selectCh]);
    }
    else
    {
        if (buttonStates == ButtonCondition::UA)
        {
            stepTriggerWork[inputStep] = 1;
            inputStep += 1;
            if (inputStep > STEP_COUNT - 1)
            {
                confirmStepTrigger(inputStep, isRec, selectCh);
            }
        }
        else if (buttonStates == ButtonCondition::UB)
        {
            stepTriggerWork[inputStep] = 0;
            inputStep += 1;
            if (inputStep > STEP_COUNT - 1)
            {
                confirmStepTrigger(inputStep, isRec, selectCh);
            }
        }
        else if (buttonStates == ButtonCondition::UMODE)
        {
            stepTriggerWork[inputStep] = -1;
            confirmStepTrigger(inputStep, isRec, selectCh);
        }
        else if (buttonStates == ButtonCondition::URE)
        {
            stepTriggerWork[inputStep] = 2;
            inputStep += 1;
            if (inputStep > STEP_COUNT - 1)
            {
                confirmStepTrigger(inputStep, isRec, selectCh);
            }
        }
        else if (buttonStates == ButtonCondition::HM_URE)
        {
        }
        else if (buttonStates == ButtonCondition::HA_HB)
        {
        }
        else if (buttonStates == ButtonCondition::HA)
        {
        }
        else if (buttonStates == ButtonCondition::HB)
        {
        }
        else if (buttonStates == ButtonCondition::HM)
        {
        }
        else if (buttonStates == ButtonCondition::NONE)
        {
        }

        rgbLedControl.setBLevel(inputStep == 0 ? 8 : (inputStep % 4) > 0 ? 4
                                                                          : 6);
    }
}

void confirmCVSequence(int16_t &inputStep, bool &isRec)
{
    inputStep = 0;
    isRec = false;
    for (int i = 0; i < STEP_COUNT; ++i)
    {
        cvSequence[i] = cvSequenceWork[i];
        cvSequenceWork[i] = 7;
    }
}

void receptOperationCV(uint16_t buttonStates, int8_t encValue)
{
    static bool isRec = false;
    static int16_t inputStep = 0;

    if (isRec == false)
    {
        if (buttonStates == ButtonCondition::UA)
        {
        }
        else if (buttonStates == ButtonCondition::UB)
        {
        }
        else if (buttonStates == ButtonCondition::UMODE)
        {
            inputStep = 0;
            isRec = true;
        }
        else if (buttonStates == ButtonCondition::URE)
        {
            currentStep = 0;
            cvSequenceCurrent = 0;
            for (int i = 0; i < OUT_COUNT; ++i)
            {
                stepTriggerCurrent[i] = 0;
            }
        }
        else if (buttonStates == ButtonCondition::HM_URE)
        {
            cvInputMode = false;
        }
        else if (buttonStates == ButtonCondition::HA_HB)
        {
        }
        else if (buttonStates == ButtonCondition::HA)
        {
        }
        else if (buttonStates == ButtonCondition::HB)
        {
        }
        else if (buttonStates == ButtonCondition::HM)
        {
        }
        else if (buttonStates == ButtonCondition::NONE)
        {
        }

        rgbLedControl.setRLevel(currentStep == 0 || currentStep == (STEP_COUNT - 1) ? 11 : 0);
        rgbLedControl.setMenuColor(RGBLEDPWMControl::MenuColor::CYAN);
        rgbLedControl.setMenuColorLevel(4);
    }
    else
    {
        if (buttonStates == ButtonCondition::UA)
        {
            inputStep += 1;
            if (inputStep > STEP_COUNT - 1)
            {
                confirmCVSequence(inputStep, isRec);
            }
        }
        else if (buttonStates == ButtonCondition::UB)
        {
            inputStep -= 1;
            if (inputStep > STEP_COUNT - 1)
            {
                confirmCVSequence(inputStep, isRec);
            }
        }
        else if (buttonStates == ButtonCondition::UMODE)
        {
            cvSequenceWork[inputStep] = -1;
            confirmCVSequence(inputStep, isRec);
        }
        else if (buttonStates == ButtonCondition::URE)
        {
        }
        else if (buttonStates == ButtonCondition::HM_URE)
        {
        }
        else if (buttonStates == ButtonCondition::HA_HB)
        {
        }
        else if (buttonStates == ButtonCondition::HA)
        {
        }
        else if (buttonStates == ButtonCondition::HB)
        {
        }
        else if (buttonStates == ButtonCondition::HM)
        {
        }
        else if (buttonStates == ButtonCondition::NONE)
        {
            cvSequenceWork[inputStep] = constrain(cvSequenceWork[inputStep] + encValue, 0, 14);
        }

        int16_t value = cvSequenceWork[inputStep];
        const RGBLEDPWMControl::MenuColor cols[7] = {
            RGBLEDPWMControl::MenuColor::GREEN,
            RGBLEDPWMControl::MenuColor::BLUE,
            RGBLEDPWMControl::MenuColor::GREEN,
            RGBLEDPWMControl::MenuColor::YELLOW,
            RGBLEDPWMControl::MenuColor::RED,
            RGBLEDPWMControl::MenuColor::GREEN,
            RGBLEDPWMControl::MenuColor::BLUE
        };
        const int8_t levels[3] = {4, 8, 11};

        rgbLedControl.setMenuColor(cols[value % 7]);
        rgbLedControl.setMenuColorLevel(levels[value / 7]);
    }
}

//////////////////////////////////////////

void edgeCallback(uint gpio, uint32_t events)
{
    if (gpio == CLOCK && useInternalClock == false)
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
    else if (gpio == DATA)
    {
        if (events & GPIO_IRQ_EDGE_RISE)
        {
            dataEdgeLatch = true;
        }
        else if (events & GPIO_IRQ_EDGE_FALL)
        {
            dataEdgeLatch = false;
        }
    }
}

void interruptPWM()
{
    pwm_clear_irq(interruptSliceNum);
}

void setup()
{
    analogReadResolution(ADC_BIT);
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
    clockIn.init(CLOCK);
    resetIn.init(RESET);
    dataIn.init(DATA);
    clockEdge.init(CLOCK); // clockエッジ期間計測のみで利用
    triggerOutManager.init();

    rgbLedControl.init(20000, PWM_BIT, LED_R, LED_G, LED_B);
    rgbLedControl.setMenuColor(menuColor);

    systemConfig.initEEPROM();
    systemConfig.loadUserConfig();

    adcErrorCorrection.init(systemConfig.Config.vRef, systemConfig.Config.noiseFloor);

    internalClock.init(SAMPLE_FREQ, PWM_BIT);
    internalClock.setWave(MiniOsc::Wave::SQU);
    internalClock.setFreqFromBPM(133, 4);
    internalClock.setLevel(PWM_BIT);
    internalClock.start();

    quantizer.setScale(5);

    initPWM(OUT_CV, PWM_RESO);

    sleep_ms(100);
    // Adjusting Mode

    initPWMIntr(PWM_INTR_PIN, interruptPWM, &interruptSliceNum, SAMPLE_FREQ, INTR_PWM_RESO, CPU_CLOCK);

    gpio_set_irq_enabled(CLOCK, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(RESET, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(DATA, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_callback(edgeCallback);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

void loop()
{
    int8_t encValue = enc.getDirection();
    int16_t dataInValue = dataIn.analogReadDirectFast();
    // int16_t resetInValue = resetIn.analogReadDirectFast();
    // int16_t clockInValue = clockIn.analogReadDirectFast();

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

    if (cvInputMode == false)
    {
        receptOperationTriggers(buttonStates, encValue);
    }
    else
    {
        receptOperationCV(buttonStates, encValue);
    }

    rgbLedControl.update();
    tight_loop_contents();
    sleep_ms(10);
}
