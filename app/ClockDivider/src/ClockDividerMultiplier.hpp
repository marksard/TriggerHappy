#pragma once

#include <stdint.h>
#include "pico/stdlib.h"

class ClockDividerMultiplier
{
public:
    static constexpr uint8_t NUM_CHANNELS = 6;
    static constexpr uint64_t PHASE_SCALE = 0x100000000ULL;

    enum class PulseMode : uint8_t
    {
        TRIGGER,
        GATE_50
    };

    struct Channel
    {
        static constexpr uint8_t FACTORS[] = {2, 3, 4, 5, 6, 7, 8, 12, 16, 32};
        static constexpr uint8_t NUM_FACTORS = sizeof(FACTORS) / sizeof(FACTORS[0]);
        uint8_t factorIndex = 0;
        bool multiply = false;
        uint8_t factor = 2;

        PulseMode pulseMode = PulseMode::GATE_50;

        uint32_t phase = 0;
        uint32_t lastUpdateUs = 0;

        bool outputState = false;
        uint32_t pulseOffTimeUs = 0;

        void addFactor(int8_t delta)
        {
            factorIndex = constrain((int8_t)factorIndex + delta, 0, NUM_FACTORS - 1);
            factor = FACTORS[factorIndex];
        }

        void toggleMultiply()
        {
            multiply = !multiply;
        }

        uint8_t getFactorIndex() const
        {
            return factorIndex;
        }

        void setFactorIndex(uint8_t index)
        {
            factorIndex = constrain(index, 0, NUM_FACTORS - 1);
            factor = FACTORS[factorIndex];
        }

        void addPulseMode(int8_t delta)
        {
            pulseMode = (PulseMode)constrain((int8_t)pulseMode + delta, (int8_t)PulseMode::TRIGGER, (int8_t)PulseMode::GATE_50);
        }
    };

    Channel channels[NUM_CHANNELS];

    void init()
    {
        uint32_t now = time_us_32();

        lastClockUs = now;

        for (auto &ch : channels)
        {
            ch.phase = 0;
            ch.lastUpdateUs = now;
            ch.outputState = false;
            ch.pulseOffTimeUs = 0;
        }
    }

    void allAddPulseMode(int8_t delta)
    {
        for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++)
        {
            channels[ch].addPulseMode(delta);
        }
    }

    void setTriggerWidthUs(uint32_t us)
    {
        triggerWidthUs = us;
    }

    void onResetRise()
    {
        resetPending = true;
    }

    void onClockRise()
    {
        uint32_t now = time_us_32();

        clockRunning = true;

        if (lastClockUs != 0)
        {
            uint32_t period = now - lastClockUs;

            if (period > 100)
            {
                clockPeriodUs = period;
            }
        }

        lastClockUs = now;

        if (resetPending)
        {
            resetPending = false;

            for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++)
            {
                channels[ch].phase = 0;
                channels[ch].lastUpdateUs = now;

                if (channels[ch].outputState)
                {
                    channels[ch].outputState = false;
                    onOutputLow(ch);
                }
            }
        }

        for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++)
        {
            auto &c = channels[ch];

            if (c.multiply)
            {
                uint32_t intervalUs =
                    clockPeriodUs / c.factor;

                fireEvent(ch, now, intervalUs);

                // Hard Sync
                c.phase = 0;
                c.lastUpdateUs = now;
            }
            else
            {
                uint64_t increment =
                    PHASE_SCALE / c.factor;

                uint64_t sum =
                    (uint64_t)c.phase +
                    increment;

                uint32_t overflowCount =
                    sum >> 32;

                c.phase =
                    (uint32_t)sum;

                uint32_t intervalUs =
                    clockPeriodUs * c.factor;

                for (uint32_t i = 0;
                     i < overflowCount;
                     i++)
                {
                    fireEvent(
                        ch,
                        now,
                        intervalUs);
                }
            }
        }
    }

    void update()
    {
        uint32_t now = time_us_32();

        //
        // Clock timeout
        //
        if (clockRunning)
        {
            uint32_t timeoutUs =
                clockPeriodUs * 2;

            if ((uint32_t)(now - lastClockUs) > timeoutUs)
            {
                clockRunning = false;

                for (uint8_t ch = 0;
                     ch < NUM_CHANNELS;
                     ch++)
                {
                    auto &c = channels[ch];

                    c.phase = 0;
                    c.lastUpdateUs = now;

                    if (c.outputState)
                    {
                        c.outputState = false;
                        onOutputLow(ch);
                    }
                }
            }
        }

        for (uint8_t ch = 0;
             ch < NUM_CHANNELS;
             ch++)
        {
            auto &c = channels[ch];

            //
            // Pulse OFF
            //
            if (c.outputState)
            {
                if ((int32_t)(now - c.pulseOffTimeUs) >= 0)
                {
                    c.outputState = false;
                    onOutputLow(ch);
                }
            }

            //
            // Multiply DDS
            //
            if (!c.multiply)
                continue;

            if (!clockRunning)
                continue;

            uint32_t dt =
                now - c.lastUpdateUs;

            c.lastUpdateUs = now;

            uint64_t increment =
                (PHASE_SCALE *
                 (uint64_t)c.factor *
                 (uint64_t)dt) /
                clockPeriodUs;

            uint64_t sum =
                (uint64_t)c.phase +
                increment;

            uint32_t overflowCount =
                sum >> 32;

            c.phase =
                (uint32_t)sum;

            uint32_t intervalUs =
                clockPeriodUs / c.factor;

            for (uint32_t i = 0;
                 i < overflowCount;
                 i++)
            {
                fireEvent(
                    ch,
                    now,
                    intervalUs);
            }
        }
    }

protected:
    virtual void onOutputHigh(uint8_t ch)
    {
    }

    virtual void onOutputLow(uint8_t ch)
    {
    }

private:
    uint32_t triggerWidthUs = 5000;
    uint32_t clockPeriodUs = 500000;
    uint32_t lastClockUs = 0;

    bool resetPending = false;
    bool clockRunning = false;

    void fireEvent(
        uint8_t ch,
        uint32_t now,
        uint32_t intervalUs)
    {
        auto &c = channels[ch];

        switch (c.pulseMode)
        {
        case PulseMode::TRIGGER:
        {
            c.outputState = true;

            c.pulseOffTimeUs =
                now + triggerWidthUs;

            onOutputHigh(ch);

            break;
        }

        case PulseMode::GATE_50:
        {
            c.outputState = true;

            c.pulseOffTimeUs =
                now + (intervalUs / 2);

            onOutputHigh(ch);

            break;
        }
        }
    }
};