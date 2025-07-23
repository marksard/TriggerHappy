/*!
 * ShiftRegister class
 * Copyright 2025 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>
#include <hardware/pwm.h>

#include "../../common/lib/EdgeChecker.hpp"
#include "../../common/lib/Quantizer.hpp"
#include "../../common/lib/RandomFast.hpp"
#include "../../common/triggerhappy_gpio.h"
#include "../../common/triggerhappy_basicconfig.h"
#include "../../common/TriggerOutManager.hpp"

class ShiftRegister
{
public:
    ShiftRegister()
        : _shiftRegisterBitsIndex(0), _r2rScale(1), _r2rOctMax(2)
    {
    }

    inline void init(TriggerOutManager *triggerOutManager, EdgeChecker *clockEdge, Quantizer *quantizer)
    {
        reset();
        _pTriggerOutManager = triggerOutManager;
        _pClockEdge = clockEdge;
        _pQuantizer = quantizer;
        _pQuantizer->setScale(_r2rScaleIndex[_r2rScale]);
        _rnd.randomSeed(millis() + micros());
        _randomDataMode = false;
        _rotate = false;
    }

    inline void reset()
    {
        for (uint8_t i = 0; i < SHIFT_REGISTER_BITSIZE; ++i)
            _shiftRegisterValues[i] = 0;
    }

    inline void process(bool useInternalClock, bool isReady, bool isClockHigh, bool *pResetEdgeLatch, bool *pDataEdgeLatch)
    {
        int duration = _pClockEdge->getDurationMills();
        duration = map(_pTriggerOutManager->getDuration(), 0, 100, 0, duration);
        
        if (isReady)
        {
            _rnd.randomSeed(micros());
            if (*pResetEdgeLatch)
            {
                *pResetEdgeLatch = false;
                reset();
            }
            uint16_t _lastValue = _shiftRegisterValues[SHIFT_REGISTER_BITSIZE - 1]; 
            uint16_t _nextCVValue = _rnd.getRandom16(256);
            bool _nextDataEdge = _rnd.getRandom16(2) >= 1;
            for (int i = SHIFT_REGISTER_BITSIZE - 1; i > 0; --i)
            {
                _shiftRegisterValues[i] = _shiftRegisterValues[i - 1];
            }

            for (int i = 0; i < (useInternalClock ? OUT_COUNT - 1 : OUT_COUNT); ++i)
            {
                bool out = _pTriggerOutManager->out(i)->getTriggerGate(0, 1);
            }

            if (_rotate)
            {
                _shiftRegisterValues[0] = _lastValue;
            }
            else if (_randomDataMode)
            {
                _shiftRegisterValues[0] = _nextDataEdge ? _nextCVValue : 0;
            }
            else {
                _shiftRegisterValues[0] = 0;
            }

            outR2R();
        }

        if (_rotate == false && _randomDataMode == false && isClockHigh && *pDataEdgeLatch) {
            _shiftRegisterValues[0] = 1;
            if (*pDataEdgeLatch)
            {
                *pDataEdgeLatch = false;
            }
        }

        for (int i = 0; i < (useInternalClock ? OUT_COUNT - 1 : OUT_COUNT); ++i)
        {
            _pTriggerOutManager->out(i)->setDuration(duration);
            bool trig = _shiftRegisterValues[_shiftRegisterBits[_shiftRegisterBitsIndex][i] - 1] != 0;
            bool out = _pTriggerOutManager->out(i)->getTriggerGate(trig, _pTriggerOutManager->isGateMode() ? 0 : 1);
            _pTriggerOutManager->out(i)->set(out);
        }

        _pTriggerOutManager->process();
    }

    inline void addScale(int8_t encValue)
    {
        if (encValue == 0)
            return;
        _r2rScale = constrain(_r2rScale + encValue, 0, 1);
        _pQuantizer->setScale(_r2rScaleIndex[_r2rScale]);
    }

    inline uint8_t getScale() { return _r2rScale; }

    // 各種setter/getter
    inline void addBitsIndex(int8_t encValue) { if (encValue == 0) return; _shiftRegisterBitsIndex = constrain(_shiftRegisterBitsIndex + encValue, 0, _shiftRegisterBitsSize - 1); }
    inline void addR2ROctMax(int8_t encValue) { if (encValue == 0) return; _r2rOctMax = constrain(_r2rOctMax + encValue, 1, 5); }
    inline void setBitsIndex(uint8_t value) { _shiftRegisterBitsIndex = value; }
    inline uint8_t getBitsIndex() const { return _shiftRegisterBitsIndex; }
    inline uint8_t getBitsSize() const { return _shiftRegisterBitsSize; }
    inline void setR2ROctMax(uint8_t octMax) { _r2rOctMax = octMax; }
    inline uint8_t getR2ROctMax() const { return _r2rOctMax; }
    inline void setRandomDataMode(bool mode) { _randomDataMode = mode; }
    inline bool getRandomDataMode() const { return _randomDataMode; }
    inline void setRotate(bool rotate) { _rotate = rotate; }
    inline bool getRotate() const { return _rotate; }

private:
    void outR2R()
    {
        uint16_t r2rOut = 1;
        if (_randomDataMode)
        {
            r2rOut = _shiftRegisterValues[0];
        }
        else
        {
            for (int i = 0; i < SHIFT_REGISTER_BITSIZE; ++i)
                r2rOut += (_shiftRegisterValues[i] != 0 ? 1 : 0) << i;
        }

        uint16_t semi = map(r2rOut, 0, 255, 0, (7 * _r2rOctMax));
        uint16_t r2rCV = _pQuantizer->Quantize(semi);
        r2rCV = constrain(r2rCV - PWMCVDCOutputErrorLUT[semi], 0, PWM_RESO - 1);
        pwm_set_gpio_level(OUT_CV, r2rCV);
    }

private:
    static constexpr uint8_t SHIFT_REGISTER_BITSIZE = 8;
    uint16_t _shiftRegisterValues[SHIFT_REGISTER_BITSIZE];
    const uint8_t _shiftRegisterBits[6][OUT_COUNT] = {
        {1, 2, 3, 4, 5, 6},
        {1, 3, 5, 7, 1, 3},
        {3, 4, 5, 6, 7, 8},
        {8, 7, 6, 5, 4, 3},
        {7, 5, 3, 1, 7, 5},
        {6, 5, 4, 3, 2, 1},
    };
    static const uint8_t _shiftRegisterBitsSize = sizeof(_shiftRegisterBits) / sizeof(_shiftRegisterBits[0]);
    uint8_t _shiftRegisterBitsIndex;
    uint8_t _r2rScale;
    const int8_t _r2rScaleIndex[2] = {0, 5};
    uint8_t _r2rOctMax;
    RandomFast _rnd;
    bool _randomDataMode;
    bool _rotate;

    // 外部依存
    TriggerOutManager *_pTriggerOutManager;
    EdgeChecker *_pClockEdge;
    Quantizer *_pQuantizer;
};