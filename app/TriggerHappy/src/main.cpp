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
#include "../../common/lib/Button.hpp"
#include "../../common/lib/SmoothAnalogRead.hpp"
#include "../../common/lib/RotaryEncoder.hpp"
#include "../../common/lib/ADCErrorCorrection.hpp"
#include "../../common/lib/RGBLEDPWMControl.hpp"
#include "../../common/lib/EepRomConfigIO.hpp"
#include "../../common/triggerhappy_gpio.h"
#include "../../common/triggerhappy_basicconfig.h"
#include "../../common/SystemConfig.hpp"

#include "../../common/lib/pwm_wrapper.h"
#include "../../common/lib/RandomFast.hpp"
#include "../../common/lib/EdgeChecker.hpp"
#include "../../common/lib/TriggerOut.hpp"
#include "../../common/lib/Quantizer.hpp"
#include "../../common/lib/Euclidean.hpp"
#include "../../common/lib/MiniOsc.hpp"
#include "../../common/TriggerOutManager.hpp"

#include "StepSeqModel.hpp"
#include "ClockDivider.hpp"
#include "ShiftRegister.hpp"
#include "StepSeqEuclid.hpp"

enum Algorithm
{
    CLOCK_DIVIDER,
    SHIFT_REGISTER,
    STEPSEQ_EUCLID,
    MAX
};

enum ButtonCondition {
    // 各ボタンの押下状態と各ボタンの組み合わせ
    // Button State: 0:None 1:Button down 2:Button up 3:Holding 4:Holded
    // U: button up
    // D: button down
    // H: holging
    // L: holded (leaved)
    // 0xMABR (Mode, A, B, RE(RotaryEncoder) button)
    NONE =   0x0000,
    UA =     0x0200,
    UB =     0x0020,
    UMODE =  0x2000,
    URE =    0x0002,
    HA =     0x0300,
    HB =     0x0030,
    HM =     0x3000,
    HA_HB =  0x0330,
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
static RGBLEDPWMControl::MenuColor menuColor = RGBLEDPWMControl::MenuColor::RED;
static ADCErrorCorrection adcErrorCorrection;

// gpio割り込み
static volatile bool clockEdgeLatch = false;
static volatile bool dataEdgeLatch = false;
static volatile bool resetEdgeLatch = false;
static EdgeChecker clockEdge;

// UIほか
static Algorithm algorithm = Algorithm::CLOCK_DIVIDER;
static uint8_t algoSubModeIndex = 0;
static bool requestGenerateSequence = false;

// 機能
static ClockDivider clockDivider;
static ShiftRegister shiftRegister;
static StepSeqEuclid stepSeqEuclid;
static TriggerOutManager triggerOutManager;
static Quantizer quantizer(PWM_RESO);
static MiniOsc internalClock;
static volatile bool useInternalClock = false;
static bool settingMode = false;
static EEPROMConfigIO<SystemConfig> systemConfig(0);

// test
static MiniOsc miniOsc;
static int8_t courseIndex = 36;
static RandomFast randomFast;
static volatile bool swapCVandOSC = false;

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

inline bool isReady()
{
    return clockEdgeLatch;
}

void changeAlgorithm(uint8_t addValue)
{
    if (addValue == 0)
        return;

    algorithm = (Algorithm)constrainCyclic(algorithm + addValue, 0, (int)Algorithm::MAX - 1);

    rgbLedControl.resetFreq();
    rgbLedControl.resetLevel();

    switch (algorithm)
    {
    case Algorithm::CLOCK_DIVIDER:
        algoSubModeIndex = 0;
        menuColor = RGBLEDPWMControl::MenuColor::RED;
        rgbLedControl.setMenuColor(menuColor);
        triggerOutManager.initPins();
        break;
    case Algorithm::SHIFT_REGISTER:
        algoSubModeIndex = 0;
        menuColor = shiftRegister.getRandomDataMode() ? RGBLEDPWMControl::MenuColor::WHITE : RGBLEDPWMControl::MenuColor::GREEN;
        rgbLedControl.setMenuColor(menuColor);
        triggerOutManager.initPins();
        shiftRegister.reset();
        break;
    case Algorithm::STEPSEQ_EUCLID:
        algoSubModeIndex = 0;
        menuColor = RGBLEDPWMControl::MenuColor::BLUE;
        rgbLedControl.setMenuColor(menuColor);
        triggerOutManager.initPins();
        break;
    default:
        menuColor = RGBLEDPWMControl::MenuColor::WHITE;
        rgbLedControl.setMenuColor(menuColor);
        break;
    }
}

void changeSettingMode()
{
    settingMode = settingMode ? false : true;
    if (settingMode)
    {
        rgbLedControl.setMenuColor(RGBLEDPWMControl::MenuColor::BLACK);
    }
    else
    {
        rgbLedControl.setMenuColor(menuColor);
    }
}

void resetOrStopStepSeq()
{
    if (useInternalClock)
    {
        if (internalClock.isStart())
        {
            internalClock.stop();
            stepSeqEuclid.resetPlayStep();
        }
        else
        {
            internalClock.start();
        }
    }
    else
    {
        stepSeqEuclid.resetPlayStep();
    }
}

//////////////////////////////////////////

void receptOperationClockDivider(uint16_t buttonStates, int8_t encValue)
{
    if (buttonStates == ButtonCondition::UA)
    {
        clockDivider.resetClockCount();
        resetOrStopStepSeq();
    }
    else if (buttonStates == ButtonCondition::UB)
    {
        clockDivider.addClockCount();
        stepSeqEuclid.nextPlayStep();
    }
    else if (buttonStates == ButtonCondition::UMODE)
    {
    }
    else if (buttonStates == ButtonCondition::URE)
    {
        changeAlgorithm(1);
    }
    else if (buttonStates == ButtonCondition::HM_URE)
    {
        changeSettingMode();
    }
    else if (buttonStates == ButtonCondition::HA_HB)
    {
        if (!requestGenerateSequence)
        {
            requestGenerateSequence = true;
            rgbLedControl.ignoreMenuColor(false);
            rgbLedControl.resetLevel();
            rgbLedControl.setBlink();
        }
    }
    else if (buttonStates == ButtonCondition::HA)
    {
        clockDivider.addDivisionsIndex(encValue);
        rgbLedControl.ignoreMenuColor(true);
        rgbLedControl.setBlink();
        rgbLedControl.setRainbowLevel(clockDivider.getDivisionsIndex(), 0, clockDivider.getDivisionsSize() - 1);
    }
    else if (buttonStates == ButtonCondition::HB)
    {
        triggerOutManager.addDurationMode(encValue);
        rgbLedControl.ignoreMenuColor(true);
        rgbLedControl.setBlink();
        rgbLedControl.setRainbowLevel(triggerOutManager.getDurationMode(), 0, triggerOutManager.getDurationsSize() - 1);
    }
    else if (buttonStates == ButtonCondition::HM)
    {
        stepSeqEuclid.getOctaveAdder().add(encValue);
        rgbLedControl.ignoreMenuColor(true);
        rgbLedControl.setBlink();
        rgbLedControl.setRainbowLevel(stepSeqEuclid.getOctaveAdder().get(), -1, stepSeqEuclid.getOctaveAdder().getDiff());
    }
    else if (buttonStates == ButtonCondition::NONE)
    {
        stepSeqEuclid.getKeyStep().setLimit(stepSeqEuclid.getKeyStep().getMin(),
                              stepSeqEuclid.getKeyStep().getMax() + encValue);
        uint8_t level = ((stepSeqEuclid.getKeyStep().get() == stepSeqEuclid.getKeyStep().getMin() ||
                            stepSeqEuclid.getKeyStep().get() == 1)
                                ? 11
                                : 0);
        rgbLedControl.ignoreMenuColor(false);
        rgbLedControl.resetFreq();
        rgbLedControl.setGLevel(level);
        uint8_t bLevel = (clockDivider.getClockCount() == 0 ? 11 : map(clockDivider.getClockCount(), 0, clockDivider.getResetCount() - 1, 0, 9));
        rgbLedControl.setBLevel(bLevel);
    }
}

void receptOperationShiftRegister(uint16_t buttonStates, int8_t encValue)
{
    if (buttonStates == ButtonCondition::UA)
    {
        resetOrStopStepSeq();
        resetEdgeLatch = true;
    }
    else if (buttonStates == ButtonCondition::UB)
    {
        dataEdgeLatch = true;
    }
    else if (buttonStates == ButtonCondition::UMODE)
    {
        shiftRegister.setRandomDataMode(!shiftRegister.getRandomDataMode());
        if (shiftRegister.getRandomDataMode())
        {
            rgbLedControl.setMenuColor(RGBLEDPWMControl::MenuColor::WHITE);
        }
        else
        {
            rgbLedControl.setMenuColor(RGBLEDPWMControl::MenuColor::GREEN);
        }
    }
    else if (buttonStates == ButtonCondition::URE)
    {
        changeAlgorithm(1);
    }
    else if (buttonStates == ButtonCondition::HM_URE)
    {
        changeSettingMode();
    }
    else if (buttonStates == ButtonCondition::HA_HB)
    {
    }
    else if (buttonStates == ButtonCondition::HA)
    {
        shiftRegister.addBitsIndex(encValue);
        rgbLedControl.ignoreMenuColor(true);
        rgbLedControl.setBlink();
        rgbLedControl.setRainbowLevel(shiftRegister.getBitsIndex(), 0, shiftRegister.getBitsSize() - 1);
    }
    else if (buttonStates == ButtonCondition::HB)
    {
        triggerOutManager.addDurationMode(encValue);
        rgbLedControl.ignoreMenuColor(true);
        rgbLedControl.setBlink();
        rgbLedControl.setRainbowLevel(triggerOutManager.getDurationMode(), 0, triggerOutManager.getDurationsSize() - 1);
    }
    else if (buttonStates == ButtonCondition::HM)
    {
        shiftRegister.addR2ROctMax(encValue);
        rgbLedControl.ignoreMenuColor(true);
        rgbLedControl.setBlink();
        rgbLedControl.setRainbowLevel(shiftRegister.getR2ROctMax(), 0, 5);
    }
    else if (buttonStates == ButtonCondition::NONE)
    {
        rgbLedControl.resetFreq();
        rgbLedControl.ignoreMenuColor(false);
        if (encValue > 0 && !shiftRegister.getRotate())
        {
            shiftRegister.setRotate(true);
        }
        else if (encValue < 0 && shiftRegister.getRotate())
        {
            shiftRegister.setRotate(false);
        }
        rgbLedControl.setRLevel(shiftRegister.getRotate() ? 11 : 0);
        rgbLedControl.setGLevel(clockEdge.getValue() ? 11 : 0);
    }
}

void receptOperationStepSeqEuclid(uint16_t buttonStates, int8_t encValue)
{
    if (buttonStates == ButtonCondition::UA)
    {
        resetOrStopStepSeq();
        stepSeqEuclid.resetEuclid();
    }
    else if (buttonStates == ButtonCondition::UB)
    {
    }
    else if (buttonStates == ButtonCondition::UMODE)
    {
        algoSubModeIndex = constrainCyclic(algoSubModeIndex + 1, 0, 3);
        switch (algoSubModeIndex)
        {
        case 0:
            rgbLedControl.setMenuColor(RGBLEDPWMControl::MenuColor::BLUE);
            break;
        case 1:
            rgbLedControl.setMenuColor(RGBLEDPWMControl::MenuColor::CYAN);
            break;
        case 2:
            rgbLedControl.setMenuColor(RGBLEDPWMControl::MenuColor::MAGENTA);
            break;
        case 3:
            rgbLedControl.setMenuColor(RGBLEDPWMControl::MenuColor::YELLOW);
            break;
        default:
            break;
        }
    }
    else if (buttonStates == ButtonCondition::URE)
    {
        changeAlgorithm(1);
    }
    else if (buttonStates == ButtonCondition::HM_URE)
    {
        changeSettingMode();
    }
    else if (buttonStates == ButtonCondition::HA_HB)
    {
        if (!requestGenerateSequence)
        {
            requestGenerateSequence = true;
            rgbLedControl.ignoreMenuColor(false);
            rgbLedControl.resetLevel();
            rgbLedControl.setBlink();
        }
    }
    else if (buttonStates == ButtonCondition::HA)
    {
        rgbLedControl.ignoreMenuColor(true);
        rgbLedControl.setBlink();
        if (algoSubModeIndex == 0)
        {
            stepSeqEuclid.getGateStep().setLimit(stepSeqEuclid.getGateStep().getMin(),
                                   stepSeqEuclid.getGateStep().getMax() + encValue);
            rgbLedControl.setRainbowLevel(stepSeqEuclid.getGateStep().getMax(), 1, DEF_MAX_STEP_M1);
        }
        else
        {
            int index = algoSubModeIndex - 1;
            stepSeqEuclid.generateEuclidForChangeOnset(encValue, index);
            rgbLedControl.setRainbowLevel(stepSeqEuclid.getEuclidean(index).getOnsets(), 0, stepSeqEuclid.getEuclidean(index).getStepSize());
        }
    }
    else if (buttonStates == ButtonCondition::HB)
    {
        rgbLedControl.ignoreMenuColor(true);
        rgbLedControl.setBlink();
        if (algoSubModeIndex == 0)
        {
            stepSeqEuclid.getGateLenAdder().add(encValue);
            rgbLedControl.setRainbowLevel(stepSeqEuclid.getGateLenAdder().get(), (int8_t)StepSeqModel::Gate::G * -1, (int8_t)StepSeqModel::Gate::G - 1);
        }
        else
        {
            triggerOutManager.addDurationMode(encValue);
            rgbLedControl.setRainbowLevel(triggerOutManager.getDurationMode(), 0, triggerOutManager.getDurationsSize() - 1);
        }
    }
    else if (buttonStates == ButtonCondition::HM)
    {
        stepSeqEuclid.getOctaveAdder().add(encValue);
        rgbLedControl.ignoreMenuColor(true);
        rgbLedControl.setBlink();
        rgbLedControl.setRainbowLevel(stepSeqEuclid.getOctaveAdder().get(), -1, stepSeqEuclid.getOctaveAdder().getDiff());
    }
    else if (buttonStates == ButtonCondition::NONE)
    {
        rgbLedControl.ignoreMenuColor(false);
        rgbLedControl.resetFreq();
        if (algoSubModeIndex == 0)
        {
            stepSeqEuclid.getKeyStep().setLimit(stepSeqEuclid.getKeyStep().getMin(),
                                              stepSeqEuclid.getKeyStep().getMax() + encValue);
            uint8_t level = ((stepSeqEuclid.getKeyStep().get() == stepSeqEuclid.getKeyStep().getMin() ||
                              stepSeqEuclid.getKeyStep().get() == 1)
                                 ? 11
                                 : 0);
            rgbLedControl.setGLevel(level);
        }
        else
        {
            int index = algoSubModeIndex - 1;
            stepSeqEuclid.generateEuclidForChangeStepSize(encValue, index);
            rgbLedControl.setGLevel(stepSeqEuclid.getEuclidean(index).getCurrent() == stepSeqEuclid.getEuclidean(index).getStartPos() ? 10 : 0);
            rgbLedControl.setRLevel(stepSeqEuclid.getEuclidean(index).getCurrentOnset() ? 5 : 0);
        }
    }
}

//////////////////////////////////////////

void receptOperationSetting(uint16_t buttonStates, int8_t encValue)
{
    if (buttonStates == ButtonCondition::UA)
    {
        useInternalClock = useInternalClock ? false : true;
    }
    else if (buttonStates == ButtonCondition::UB)
    {
        stepSeqEuclid.setDiv(constrainCyclic(stepSeqEuclid.getDiv() + 1, 1, 4));
    }
    else if (buttonStates == ButtonCondition::UMODE)
    {
    }
    else if (buttonStates == ButtonCondition::URE)
    {
    }
    else if (buttonStates == ButtonCondition::HM_URE)
    {
        changeSettingMode();
    }
    else if (buttonStates == ButtonCondition::HA_HB)
    {
    }
    else if (buttonStates == ButtonCondition::HA)
    {
        stepSeqEuclid.addScale(encValue);
        shiftRegister.addScale(encValue);
        rgbLedControl.ignoreMenuColor(true);
        rgbLedControl.setBlink();
        rgbLedControl.setRainbowLevel(stepSeqEuclid.getScale(), 0, 1);
    }
    else if (buttonStates == ButtonCondition::HB)
    {
        stepSeqEuclid.addOctaveMax(encValue);
        rgbLedControl.ignoreMenuColor(true);
        rgbLedControl.setBlink();
        rgbLedControl.setRainbowLevel(stepSeqEuclid.getOctaveMax(), 0, stepSeqEuclid.OCTAVE_MAX);
    }
    else if (buttonStates == ButtonCondition::HM)
    {
    }
    else if (buttonStates == ButtonCondition::NONE)
    {
        internalClock.setFreqFromBPM(internalClock.getBPM() + encValue, 4);
        rgbLedControl.ignoreMenuColor(false);
        rgbLedControl.resetFreq();
        if (useInternalClock) rgbLedControl.setRLevel(internalClock.getWaveValue() ? 11 : 0);
    }
}

//////////////////////////////////////////

static uint16_t outCVTestSemiTone = 0;
static int16_t vOctValue = 0;

void checkVOct()
{
    // voct誤差表示用
    static int16_t lastVOctValue = 0;
    static bool flag = false;
    static uint16_t count = 0;
    static int32_t vOctMean = 0;
    if (vOctValue > lastVOctValue + 30 || vOctValue < lastVOctValue - 30)
    {
        flag = true;
    }
    lastVOctValue = vOctValue;
    if (flag)
    {
        vOctMean += vOctValue;
        count++;
        if (count >= 128)
        {
            vOctMean = vOctMean >> 7;
            Serial.print(vOctMean);
            Serial.println("");
            flag = false;
            count = 0;
            vOctMean = 0;
            // outputSemiSelect++;
            // if (outputSemiSelect > 60)
            // {
            //     outputSemiSelect = 0;
            // }
        }
    }
    sleep_ms(10);
}

void processAdjusting(int16_t dataInValue)
{
    vOctValue = dataInValue;
    float voctPowV = adcErrorCorrection.voctPow(dataInValue);
    float vOctFreq = miniOsc.getFreqFromNoteIndex(courseIndex) * voctPowV;

    miniOsc.setFrequency(vOctFreq);

    int16_t voct = outCVTestSemiTone * quantizer.VoltPerTone;
    voct = constrain(voct - PWMCVDCOutputErrorLUT[outCVTestSemiTone], 0, PWM_RESO - 1);
    pwm_set_gpio_level(swapCVandOSC ? OUT5 : OUT_CV, voct);

    static uint16_t dispCount = 0;
    dispCount++;
    if (dispCount == 2000)
    {
        dispCount = 0;
        Serial.print(" vref:");
        Serial.print(adcErrorCorrection.getLastVRef(), 4);
        Serial.print(" noise:");
        Serial.print(adcErrorCorrection.getLastNoiseFloor(), 2);
        Serial.print(" dataInValue:");
        Serial.print(dataInValue);
        Serial.print(" vOctValue:");
        Serial.print(vOctValue);
        Serial.print(" vOctFreq:");
        Serial.print(vOctFreq);
        Serial.print(" voct:");
        Serial.print(voct);
        Serial.println();
    }
}

void receptOperationAdjusting(uint16_t buttonStates, int8_t encValue)
{
    if (buttonStates == ButtonCondition::UA)
    {
        swapCVandOSC = swapCVandOSC ? false : true;
    }
    else if (buttonStates == ButtonCondition::URE)
    {
        changeAlgorithm(1);
    }
    else if (buttonStates == ButtonCondition::HM_URE)
    {
        changeSettingMode();
    }
    else if (buttonStates == ButtonCondition::HA)
    {
        courseIndex = constrain(courseIndex + (encValue), 0, 127);
        rgbLedControl.setBlink();
        rgbLedControl.setRLevel((courseIndex % 12) == 0 ? 11 : 0);
    }
    else if (buttonStates == ButtonCondition::HB)
    {
        miniOsc.setWave((MiniOsc::Wave)(miniOsc.getWave() + encValue));
    }
    else if (buttonStates == ButtonCondition::NONE)
    {
        outCVTestSemiTone = constrain(outCVTestSemiTone + encValue, 0, 60);
        rgbLedControl.resetFreq();
        rgbLedControl.setRLevel((outCVTestSemiTone % 12) == 0 ? 11 : 0);
    }

    // checkVOct();
}

//////////////////////////////////////////

void calibration(float &vref, float &noiseFloor)
{
    Serial.println("VOCT Calibration");
    noiseFloor = adcErrorCorrection.getADCMax16(RESET);
    pwm_set_gpio_level(OUT_CV, 2047);
    sleep_ms(50);
    float adc = adcErrorCorrection.getADCMax16(DATA);
    Serial.print("ADC at 5V:");
    Serial.print(adc);
    if (adc >= 4093)
    {
        // 3.26989付近なので半分の電圧から推定しなおす
        pwm_set_gpio_level(OUT_CV, 1024);
        sleep_ms(50);
        adc = adcErrorCorrection.getADCMax16(DATA);
        Serial.print(" at 2.5V:");
        Serial.print(adc);
        adc *= 2;
    }
    pwm_set_gpio_level(OUT_CV, 0);
    sleep_ms(50);
    vref = adcErrorCorrection.getADC2VRef(adc);
    Serial.print(" vref:");
    Serial.print(vref, 4);
    Serial.print(" noiseFloor:");
    Serial.println(noiseFloor);

    adcErrorCorrection.generateLUT(vref, noiseFloor);
}

void edgeCallback(uint gpio, uint32_t events)
{
    if (gpio == CLOCK && useInternalClock == false)
    {
        if (events & GPIO_IRQ_EDGE_RISE)
        {
            clockEdge.updateEdge(1);
            clockEdgeLatch = true;
            clockDivider.addClockCount();
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
            clockDivider.zeroResetClockCount();
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
    switch (algorithm)
    {
    case Algorithm::CLOCK_DIVIDER:
        break;
    case Algorithm::SHIFT_REGISTER:
        break;
    case Algorithm::STEPSEQ_EUCLID:
        break;
    default:
        pwm_set_gpio_level(swapCVandOSC ? OUT_CV : OUT5, miniOsc.getWaveValue());
        break;
    }

    if (useInternalClock)
    {
        uint16_t value = internalClock.getWaveValue();
        static bool internalClockEdge = false;
        int clockTrigger = 0;
        if (internalClock.isStart() == false)
        {
            internalClockEdge = false;
            clockEdge.updateEdge(0);
        }
        else if (value != 0 && internalClockEdge == false)
        {
            internalClockEdge = true;
            clockEdge.updateEdge(1);
            clockEdgeLatch = true;
            clockDivider.addClockCount();
            int duration = internalClock.getDurationMills();
            triggerOutManager.out(5)->setDuration(duration >> 3);
            clockTrigger = 1;
        }
        else if (value == 0 && internalClockEdge == true)
        {
            internalClockEdge = false;
            clockEdge.updateEdge(0);
            clockEdgeLatch = false;
        }
        triggerOutManager.out(5)->update(clockTrigger);
    }
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

    clockDivider.init(&triggerOutManager, &clockEdge);
    shiftRegister.init(&triggerOutManager, &clockEdge, &quantizer);
    stepSeqEuclid.init(&triggerOutManager, &clockEdge, &quantizer);

    miniOsc.init(SAMPLE_FREQ, PWM_BIT);
    miniOsc.setWave(MiniOsc::Wave::SQU);
    miniOsc.setFrequency(440);
    miniOsc.setLevel(PWM_BIT);
    
    internalClock.init(SAMPLE_FREQ, PWM_BIT);
    internalClock.setWave(MiniOsc::Wave::SQU);
    internalClock.setFreqFromBPM(133, 4);
    internalClock.setLevel(PWM_BIT);
    internalClock.start();
    
    initPWM(OUT_CV, PWM_RESO);

    sleep_ms(100);
    // Adjusting Mode
    if (gpio_get(BTN_MODE) == false || gpio_get(BTN_A) == false)
    {
        algorithm = Algorithm::MAX;
        algoSubModeIndex = 0;
        menuColor = RGBLEDPWMControl::MenuColor::WHITE;
        rgbLedControl.setMenuColor(menuColor);
        triggerOutManager.reset();
        initPWM(OUT5, PWM_RESO);
    }

    // VOCT Calibration
    if (gpio_get(BTN_A) == false)
    {
        // while (!Serial)
        // {
        // }

        float vref = 0.0;
        float noiseFloor = 0.0;
        calibration(vref, noiseFloor);
        systemConfig.Config.noiseFloor = noiseFloor;
        systemConfig.Config.vRef = vref;
        systemConfig.saveUserConfig();
    }

    // ADC LOG Mode
    if (gpio_get(BTN_B) == false)
    {
        while (!Serial)
        {
        }

        float vref = 0.0;
        float noiseFloor = 0.0;
        calibration(vref, noiseFloor);
        sleep_ms(100);
        Serial.println("out,raw_adc,raw_diff,cor_adc,cor_diff");
        for (int i = 0; i < PWM_RESO; ++i)
        {
            pwm_set_gpio_level(OUT_CV, i);
            sleep_ms(1);
            int16_t raw_adc = adcErrorCorrection.getADCAvg16(DATA);
            int16_t raw_diff = (i * 2) - adcErrorCorrection.correctedInputScaleAdc(raw_adc);
            Serial.print(i * 2);
            Serial.print(",");
            Serial.print(raw_adc);
            Serial.print(",");
            Serial.print(raw_diff);
            int16_t cor_adc = (int)adcErrorCorrection.correctedAdc(raw_adc);
            int16_t cor_diff = (i * 2) - cor_adc;
            Serial.print(",");
            Serial.print(cor_adc);
            Serial.print(",");
            Serial.print(cor_diff);
            Serial.println();
        }
    }
    
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

    bool ready = isReady();
    switch (algorithm)
    {
    case Algorithm::CLOCK_DIVIDER:
        if (ready)
        {
            stepSeqEuclid.processStepSeq(&requestGenerateSequence, NULL, NULL);
            clockEdgeLatch = false;
        }
        clockDivider.process(useInternalClock, ready, dataInValue);
        break;
    case Algorithm::SHIFT_REGISTER:
        if (ready)
        {
            stepSeqEuclid.processStepSeq(&requestGenerateSequence, NULL, NULL);
            clockEdgeLatch = false;
        }
        shiftRegister.process(useInternalClock, ready, clockEdge.getValue(), (bool*)&resetEdgeLatch, (bool*)&dataEdgeLatch);
        break;
    case Algorithm::STEPSEQ_EUCLID:
        if (ready)
        {
            clockEdgeLatch = false;
        }
        if (resetEdgeLatch)
        {
            requestGenerateSequence = true;
        }
        stepSeqEuclid.process(ready, &requestGenerateSequence, dataInValue);
        break;
    default:
        processAdjusting(dataInValue);
        break;
    }

    rgbLedControl.process();

    tight_loop_contents();
    sleep_us(50);

    // static int8_t lastEncValue = 0;
    // if (encValue != 0 && encValue != lastEncValue)
    // {
    //     Serial.print("Enc:");
    //     Serial.print(encValue);
    //     Serial.println();
    // }
    // lastEncValue = encValue;
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

    if (settingMode)
    {
        receptOperationSetting(buttonStates, encValue);
        rgbLedControl.update();
        tight_loop_contents();
        sleep_ms(1);
        return;
    }
    
    switch (algorithm)
    {
    case Algorithm::CLOCK_DIVIDER:
        receptOperationClockDivider(buttonStates, encValue);
        break;
    case Algorithm::SHIFT_REGISTER:
        receptOperationShiftRegister(buttonStates, encValue);
        break;
    case Algorithm::STEPSEQ_EUCLID:
        receptOperationStepSeqEuclid(buttonStates, encValue);
        break;
    default:
        receptOperationAdjusting(buttonStates, encValue);
        break;
    }

    rgbLedControl.update();
    tight_loop_contents();
    sleep_ms(10);
}
