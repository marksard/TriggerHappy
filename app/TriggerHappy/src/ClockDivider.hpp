/*!
 * ClockDivider class
 * Copyright 2025 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>
#include <numeric>

#include "../../common/lib/EdgeChecker.hpp"
#include "../../common/triggerhappy_gpio.h"
#include "../../common/triggerhappy_basicconfig.h"
#include "../../common/TriggerOutManager.hpp"

class ClockDivider
{
public:
    ClockDivider(): _divisionsIndex(0), _resetCount(0), _clockCount(0) {}

    inline void init(
        TriggerOutManager *triggerOutManager,
        EdgeChecker *clockEdge)
    {
        _pTriggerOutManager = triggerOutManager;
        _pClockEdge = clockEdge;

        // 初期値
        _divisionsIndex = 2;
        calcDivisionResetCounts();
        _resetCount = _divisionResetCounts[_divisionsIndex];
        resetClockCount();
    }

    inline void process(bool useInternalClock, bool isReady, int16_t dataInValue)
    {
        static int16_t lastDataInCV = 0;
        int16_t divSizeM1 = _divisionsSize - 1;
        int16_t divisionsIndexWithCV = constrain(_divisionsIndex + lastDataInCV, 0, divSizeM1);
        int duration = _pClockEdge->getDurationMills();
        duration = map(_pTriggerOutManager->getDuration(), 0, 100, 0, duration);
        if (isReady)
        {
            // CVをミックス
            dataInValue = map(dataInValue, 0, ADC_RESO - 1, 0, divSizeM1);
            if (lastDataInCV != dataInValue)
            {
                lastDataInCV = dataInValue;
                divisionsIndexWithCV = constrain(_divisionsIndex + dataInValue, 0, divSizeM1);
                _resetCount = _divisionResetCounts[divisionsIndexWithCV];
            }
        }

        for (int i = 0; i < (useInternalClock ? OUT_COUNT - 1 : OUT_COUNT); ++i)
        {
            _pTriggerOutManager->out(i)->setDuration(duration);
            int16_t div = _divisions[divisionsIndexWithCV][i];
            bool trig = divide(_clockCount, div);
            bool out = _pTriggerOutManager->out(i)->getTriggerGate(div == 1 ? _pClockEdge->getValue() : trig,
                                                                   _pTriggerOutManager->isGateMode() ? 0 : 1);
            _pTriggerOutManager->out(i)->set(out);
        }

        _pTriggerOutManager->process();
    }

    inline void resetClockCount() { _clockCount = _resetCount - 1; }
    inline void zeroResetClockCount() { _clockCount = 0; }
    inline void addClockCount() { _clockCount = (_clockCount + 1) % _resetCount; }
    inline void addDivisionsIndex(int8_t encValue)
    {
        if (encValue == 0) return;
        _divisionsIndex = constrain(_divisionsIndex + encValue, 0, _divisionsSize - 1);
        _resetCount = _divisionResetCounts[_divisionsIndex];
    }
    inline uint8_t getDivisionsIndex() { return _divisionsIndex; }
    inline size_t getDivisionsSize() { return _divisionsSize; }
    inline uint64_t getClockCount() { return _clockCount; }
    inline uint64_t getResetCount() { return _resetCount; }

private:
    inline bool divide(uint64_t clockCount, int16_t div)
    {
        return (clockCount % div) < (div >> 1);
    }

    /// @brief 分割数選択テーブルでトリガーをキレイにひとまわりさせるため最小公倍数を求めたものを使う
    inline void calcDivisionResetCounts()
    {
        for (size_t row = 0; row < _divisionsSize; ++row)
        {
            uint64_t lcm = 1;
            for (size_t col = 0; col < OUT_COUNT; ++col)
            {
                lcm = std::lcm(lcm, static_cast<uint64_t>(_divisions[row][col]));
            }

            _divisionResetCounts[row] = lcm;
        }
    }

private:
    // 分割数選択テーブル。対応出力数はOUT_COUNT<=6まで
    const uint8_t _divisions[6][OUT_COUNT]{
        {2, 3, 4, 5, 6, 7},
        {2, 4, 6, 8, 10, 12},
        {2, 4, 8, 16, 32, 64},
        {3, 4, 5, 6, 7, 8},
        {3, 5, 7, 9, 11, 13},
        {3, 6, 12, 24, 48, 96},
    };
    static const size_t _divisionsSize = sizeof(_divisions) / sizeof(_divisions[0]);
    uint64_t _divisionResetCounts[_divisionsSize];
    uint8_t _divisionsIndex;
    uint64_t _resetCount;
    uint64_t _clockCount;

    // 外部依存
    TriggerOutManager *_pTriggerOutManager;
    EdgeChecker *_pClockEdge;
};
