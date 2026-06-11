/*!
 * SyncLFO class
 * Copyright 2026 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once

#include <stdint.h>
#include "pico/stdlib.h"

class SyncLFO
{
public:
    static constexpr uint64_t PHASE_SCALE =
        0x100000000ULL;

    struct RatioInfo
    {
        bool multiply;
        uint8_t factor;
    };

    enum class RatioIndex : uint8_t
    {
        DIV128,
        DIV64,
        DIV32,
        DIV16,
        DIV12,
        DIV8,
        DIV7,
        DIV6,
        DIV5,
        DIV4,
        DIV3,
        DIV2,

        MUL1,

        MUL2,
        MUL3,
        MUL4,

        COUNT
    };

    static constexpr RatioInfo RATIO_TABLE[] =
        {
            {false, 128},
            {false, 64},
            {false, 32},
            {false, 16},
            {false, 12},
            {false, 8},
            {false, 7},
            {false, 6},
            {false, 5},
            {false, 4},
            {false, 3},
            {false, 2},

            {true, 1},

            {true, 2},
            {true, 3},
            {true, 4}};

    enum class Waveform : uint8_t
    {
        SAW,
        RAMP,
        TRIANGLE,
        SQUARE,

        COUNT
    };

    void init(uint8_t heightBit = 11)
    {
        phase = 0;
        lastClockUs = 0;
        lastUpdateUs = 0;
        pwmResoM1 = (1 << heightBit) - 1;
        indexBit = 32 - heightBit;
    }

    void addRatio(int8_t delta)
    {
        RatioIndex r = (RatioIndex)constrain((int8_t)ratioIndex + delta, 0, (int8_t)RatioIndex::COUNT - 1);
        setRatio(r);
    }

    void setRatio(RatioIndex r)
    {
        ratioIndex = r;
        multiply = RATIO_TABLE[(int8_t)r].multiply;
        factor = RATIO_TABLE[(int8_t)r].factor;
    }

    void addWaveform(int8_t delta)
    {
        waveform = (Waveform)constrain((int8_t)waveform + delta, 0, (int8_t)Waveform::COUNT - 1);
    }

    void setWaveform(Waveform wf)
    {
        waveform = wf;
    }

    void setFactor(uint8_t f)
    {
        factor = (f == 0) ? 1 : f;
    }

    void setMultiply(bool m)
    {
        multiply = m;
    }

    void onResetRise()
    {
        resetPending = true;
    }

    void onClockRise()
    {
        uint32_t now = time_us_32();

        running = true;

        if (lastClockUs != 0)
        {
            uint32_t period =
                now - lastClockUs;

            if (period > 100)
            {
                clockPeriodUs = period;
            }
        }

        lastClockUs = now;

        if (resetPending)
        {
            resetPending = false;

            phase = 0;
        }

        // if (multiply)
        // {
        //     // Hard Sync
        //     phase = 0;
        //     lastUpdateUs = now;
        // }
    }

    void update()
    {
        uint32_t now = time_us_32();

        if (running)
        {
            if ((uint32_t)(now - lastClockUs) > (clockPeriodUs * 2))
            {
                running = false;
            }
        }

        if (!running)
            return;

        uint32_t dt =
            now - lastUpdateUs;

        lastUpdateUs = now;

        uint64_t increment;

        if (multiply)
        {
            increment =
                (PHASE_SCALE *
                 factor *
                 (uint64_t)dt) /
                clockPeriodUs;
        }
        else
        {
            increment =
                (PHASE_SCALE *
                 (uint64_t)dt) /
                (clockPeriodUs * factor);
        }

        phase += (uint32_t)increment;
    }

    uint16_t getValue() const
    {
        switch (waveform)
        {
        case Waveform::SAW:
        {
            return phase >> indexBit;
        }

        case Waveform::RAMP:
        {
            return pwmResoM1 - (phase >> indexBit);
        }

        case Waveform::TRIANGLE:
        {
            uint32_t p = phase;

            if (p & 0x80000000UL)
            {
                p = 0xFFFFFFFFUL - p;
            }

            return p >> (indexBit - 1);
        }

        case Waveform::SQUARE:
        {
            return (phase & 0x80000000UL)
                       ? pwmResoM1
                       : 0;
        }
        }

        return 0;
    }

private:
    Waveform waveform =
        Waveform::TRIANGLE;

    bool multiply = false;
    uint8_t factor = 1;
    RatioIndex ratioIndex = RatioIndex::MUL1;

    uint32_t phase = 0;
    uint16_t pwmResoM1 = 0;
    uint8_t indexBit = 0;

    uint32_t clockPeriodUs = 500000;

    uint32_t lastClockUs = 0;
    uint32_t lastUpdateUs = 0;

    bool running = false;
    bool resetPending = false;
};