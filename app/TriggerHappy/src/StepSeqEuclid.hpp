/*!
 * StepSeqEuclid class
 * Copyright 2025 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>
#include <hardware/pwm.h>

#include "../../common/lib/EdgeChecker.hpp"
#include "../../common/lib/Quantizer.hpp"
#include "../../common/lib/Euclidean.hpp"
#include "../../common/triggerhappy_gpio.h"
#include "../../common/triggerhappy_basicconfig.h"
#include "../../common/TriggerOutManager.hpp"
#include "StepSeqModel.hpp"

class StepSeqEuclid
{
public:
    static constexpr int EUCLID_COUNT = 3;
    static constexpr int OCTAVE_MAX = 5;

    StepSeqEuclid() {}

    inline void init(TriggerOutManager *triggerOutManager, EdgeChecker *clockEdge, Quantizer *quantizer)
    {
        _pTriggerOutManager = triggerOutManager;
        _pClockEdge = clockEdge;
        _pQuantizer = quantizer;
        _stepSeqModel.resetSequence(StepSeqModel::Gate::_);
        _scale = 1;
        _octaveMax = 3;
        generateStepSeq();
        _euclidean[0].generate(9, 15);
        _euclidean[1].generate(7, 13);
        _euclidean[2].generate(5, 11);
    }

    inline void generateStepSeq()
    {
        _stepSeqModel.generateSequence(0, _octaveMax,
                                       StepSeqModel::Gate::S,
                                       StepSeqModel::Gate::G,
                                       StepSeqModel::Gate::_,
                                       true);

        _stepSeqModel._scaleIndex.set(_scaleIndex[_scale]);
        _stepSeqModel.keyStep.setMode(Step::Mode::Forward);
        _stepSeqModel.gateStep.setMode(Step::Mode::Forward);
        _stepSeqModel.keyStep.resetPlayStep();
        _stepSeqModel.gateStep.resetPlayStep();
        _stepSeqModel.printSeq();
    }

    inline void generateEuclidForChangeOnset(int8_t encValue, uint8_t index)
    {
        if (index >= EUCLID_COUNT)
            return;
        if (encValue == 0)
            return;
        _euclidean[index].generate(_euclidean[index].getOnsets() + encValue, _euclidean[index].getStepSize());
    }

    inline void generateEuclidForChangeStepSize(int8_t encValue, uint8_t index)
    {
        if (index >= EUCLID_COUNT)
            return;
        if (encValue == 0)
            return;
        _euclidean[index].generate(_euclidean[index].getOnsets(), _euclidean[index].getStepSize() + encValue);
    }

    int16_t divCount = 0;
    int16_t div = 1;

    inline void addDiv(int16_t envValue)
    {
        if (envValue == 0) return;
        div = constrain(div + envValue, 1, 4);
        divCount = 0;
    }

    inline void setDiv(int16_t value)
    {
        div = value;
        divCount = 0;
    }
    inline int16_t getDiv() { return div; }

    inline bool processStepSeq(bool *pRequestGenerateSequence, uint8_t *pGate, uint8_t *pAcc)
    {
        if (*pRequestGenerateSequence)
        {
            if (_stepSeqModel.keyStep.pos.getMin() == _stepSeqModel.keyStep.pos.get())
            {
                *pRequestGenerateSequence = false;
                generateStepSeq();
            }
        }

        uint8_t gate = _stepSeqModel.getPlayGate();
        uint8_t acc = _stepSeqModel.getPlayAcc();
        if (pGate)
            *pGate = gate;
        if (pAcc)
            *pAcc = acc;

        if (divCount % div != 0)
        {
            divCount++;
            // Serial.println("divcount not match");
            return false;
        }
        divCount++;
        
        uint8_t semi = _stepSeqModel.getPlayNote();
        int16_t voct = semi * _pQuantizer->VoltPerTone;
        voct = constrain(voct - PWMCVDCOutputErrorLUT[semi], 0, PWM_RESO - 1);
        pwm_set_gpio_level(OUT_CV, voct);
        _stepSeqModel.keyStep.nextPlayStep();
        _stepSeqModel.gateStep.nextPlayStep();

        return true;
    }

    inline void nextPlayStep()
    {
        _stepSeqModel.keyStep.nextPlayStep();
        _stepSeqModel.gateStep.nextPlayStep();
        divCount = 0;
    }

    inline void resetPlayStep()
    {
        _stepSeqModel.keyStep.resetPlayStep();
        _stepSeqModel.gateStep.resetPlayStep();
        divCount = 0;
    }

    inline void resetEuclid()
    {
        for (int i = 0; i < EUCLID_COUNT; ++i)
        {
            _euclidean[i].resetCurrent();
        }
    }

    inline void process(bool isReady, bool *pRequestGenerateSequence, int16_t dataInValue)
    {
        const int8_t dataInCVMin = 5;
        static int16_t lastDataInCV = dataInCVMin;
        static uint8_t stepSeqGate = 0;
        uint8_t gate = 0;
        uint8_t acc = 0;
        bool updateDuration = false;

        if (isReady)
        {
            int duration = _pClockEdge->getDurationMills();
            updateDuration = processStepSeq(pRequestGenerateSequence, &stepSeqGate, &acc);
            if (updateDuration)
            {
                int durationPercentage = _stepSeqModel.GateDuration[stepSeqGate];
                int stepSeqDuration = map(durationPercentage, 0, 100, 0, duration);
    
                gate = 1;
                _pTriggerOutManager->out(3)->setDuration(stepSeqDuration);
                _pTriggerOutManager->out(4)->setDuration(200);
            }

            int eucDuration = map(_pTriggerOutManager->getDuration(), 0, 100, 0, duration);
            for (int i = 0; i < EUCLID_COUNT; ++i)
            {
                int8_t trig = _euclidean[i].getNext();
                _pTriggerOutManager->out(i)->setDuration(eucDuration);
                bool out = _pTriggerOutManager->out(i)->getTriggerGate(trig, _pTriggerOutManager->isGateMode() ? 0 : 1);
                _pTriggerOutManager->out(i)->set(out);
            }
        }
        else
        {
            // CVをミックス
            dataInValue = map(dataInValue, 0, ADC_RESO - 1, dataInCVMin, Euclidean::EUCLID_MAX_STEPS);
            if (lastDataInCV != dataInValue)
            {
                // Serial.println("change");
                lastDataInCV = dataInValue;
                for (int i = 0; i < EUCLID_COUNT; ++i)
                {
                    int8_t onset = _stepSeqModel._rand.getRandom16(dataInValue);
                    int8_t stepSize = _stepSeqModel._rand.getRandom16(dataInValue);
                    if (onset != stepSize) 
                    {
                        _euclidean[i].generate(onset, stepSize);
                    }
                }
            }
        }

        if (stepSeqGate == StepSeqModel::Gate::_)
        {
            _pTriggerOutManager->out(3)->set(0);
        }
        else if (stepSeqGate == StepSeqModel::Gate::G)
        {
            _pTriggerOutManager->out(3)->set(1);
        }
        else
        {
            _pTriggerOutManager->out(3)->update(gate);
        }
        _pTriggerOutManager->out(4)->update(acc);

        for (int i = 0; i < EUCLID_COUNT; ++i)
        {
            _pTriggerOutManager->out(i)->update(0);
        }

        _pTriggerOutManager->process();
    }

    inline void addScale(int8_t encValue)
    {
        if (encValue == 0)
            return;
        _scale = constrain(_scale + encValue, 0, 1);
        _stepSeqModel._scaleIndex.set(_scaleIndex[_scale]);
    }

    inline uint8_t getScale() { return _scale; }

    inline void addOctaveMax(int8_t encValue)
    {
        if (encValue == 0)
            return;
        _octaveMax = constrain(_octaveMax + encValue, 1, OCTAVE_MAX);
    }

    inline uint8_t getOctaveMax() { return _octaveMax; }

    inline LimitValue<int8_t> &getOctaveAdder() { return _stepSeqModel.octaveAdder; }
    inline LimitValue<int8_t> &getKeyStep() { return _stepSeqModel.keyStep.pos; }
    inline LimitValue<int8_t> &getGateLenAdder() { return _stepSeqModel.gateLenAdder; }
    inline LimitValue<int8_t> &getGateStep() { return _stepSeqModel.gateStep.pos; }
    inline StepSeqModel &getStepSeqModel() { return _stepSeqModel; }
    inline Euclidean &getEuclidean(int idx) { return _euclidean[idx]; }

private:
    StepSeqModel _stepSeqModel;
    Euclidean _euclidean[EUCLID_COUNT];
    uint8_t _scale;
    const int8_t _scaleIndex[2] = {0, 5};
    uint8_t _octaveMax;

    TriggerOutManager *_pTriggerOutManager;
    EdgeChecker *_pClockEdge;
    Quantizer *_pQuantizer;
};
