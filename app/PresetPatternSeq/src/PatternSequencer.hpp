/*!
 * PatternSequencer class
 * Copyright 2026 marksard
 * This software is released under the MIT license.
 */

#pragma once
#include <stdint.h>
#include "pico/stdlib.h"
#include <vector>

class PatternSequencer
{
public:
    static constexpr uint8_t NUM_CHANNELS = 3;
    static constexpr uint8_t MAX_STEPS = 64;
    static constexpr uint64_t PHASE_SCALE = 0x100000000ULL;
    static constexpr uint8_t TICKS_PER_STEP = 6;

    static constexpr uint8_t TYPE_MASK = 0x0F;
    static constexpr uint8_t PROB_MASK = 0xF0;

    enum class TriggerType : uint8_t
    {
        OFF = 0,
        ON = 1,
        SUBSTEP_D2 = 2,
        SUBSTEP_D3 = 3
    };

    void init()
    {
        uint32_t now = time_us_32();
        lastClockUs = now;
        clockRunning = false;
        currentStep = 0;
        currentTick = 0;
        resetPending = false;

        srand(now);

        for (uint8_t i = 0; i < NUM_CHANNELS; i++)
        {
            channels[i].outputState = false;
            channels[i].pulseOffTimeUs = 0;
            channels[i].isStepEnabled = true;
        }

        memset(patternMatrix, 0, sizeof(patternMatrix));
    }

    void setPattern(uint8_t ch, const uint8_t *patternData, uint8_t length)
    {
        if (ch >= NUM_CHANNELS)
            return;
        uint8_t stepsToCopy = length > MAX_STEPS ? MAX_STEPS : length;
        memcpy(patternMatrix[ch], patternData, stepsToCopy);
    }

    void requieredMute(uint8_t ch, bool mute)
    {
        if (ch >= NUM_CHANNELS)
            return;
        channels[ch].requiredMute = mute;
    }

    void requieredAllOn(uint8_t ch, bool on)
    {
        if (ch >= NUM_CHANNELS)
            return;
        channels[ch].requiredAllOn = on;
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
                stepPeriodUs = period;
            }
        }
        lastClockUs = now;

        if (resetPending)
        {
            currentStep = 0;
            resetPending = false;
        }
        else
        {
            currentStep = (currentStep + 1) % MAX_STEPS;
        }

        currentTick = 0;

        evaluateProbability();

        processTickEvents(now);

        lastTickUpdateUs = now;
        tickPhase = 0;
    }

    void update()
    {
        uint32_t now = time_us_32();

        if (clockRunning)
        {
            if ((uint32_t)(now - lastClockUs) > stepPeriodUs * 2)
            {
                clockRunning = false;
                allOutputsLow();
            }
        }

        // トリガーパルスのOFF処理
        for (uint8_t i = 0; i < NUM_CHANNELS; i++)
        {
            if (channels[i].outputState)
            {
                if ((int32_t)(now - channels[i].pulseOffTimeUs) >= 0)
                {
                    channels[i].outputState = false;
                    onOutputLow(i);
                }
            }
        }

        if (!clockRunning)
            return;

        // DDSによる内部ティック更新
        uint32_t dt = now - lastTickUpdateUs;
        lastTickUpdateUs = now;

        uint64_t increment = (PHASE_SCALE * (uint64_t)TICKS_PER_STEP * (uint64_t)dt) / stepPeriodUs;
        uint64_t sum = (uint64_t)tickPhase + increment;
        uint32_t overflowCount = sum >> 32;
        tickPhase = (uint32_t)sum;

        for (uint32_t i = 0; i < overflowCount; i++)
        {
            currentTick++;
            if (currentTick < TICKS_PER_STEP)
            {
                processTickEvents(now);
            }
            else
            {
                currentTick = TICKS_PER_STEP - 1;
            }
        }
    }

protected:
    virtual void onOutputHigh(uint8_t ch) {}
    virtual void onOutputLow(uint8_t ch) {}

private:
    struct ChannelState
    {
        bool outputState = false;
        uint32_t pulseOffTimeUs = 0;
        bool isStepEnabled = true;
        bool requiredMute = false;
        bool requiredAllOn = false;
    };

    ChannelState channels[NUM_CHANNELS];
    uint8_t patternMatrix[NUM_CHANNELS][MAX_STEPS];

    uint32_t triggerWidthUs = 5000;
    uint32_t stepPeriodUs = 125000;
    uint32_t lastClockUs = 0;
    uint32_t lastTickUpdateUs = 0;
    uint32_t tickPhase = 0;

    uint8_t currentStep = 0;
    uint8_t currentTick = 0;

    bool clockRunning = false;
    bool resetPending = false;

    void evaluateProbability()
    {
        for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++)
        {
            uint8_t rawValue = patternMatrix[ch][currentStep];
            uint8_t probType = rawValue & PROB_MASK;
            uint8_t triggerType = rawValue & TYPE_MASK;

            if (triggerType == (uint8_t)TriggerType::OFF)
            {
                channels[ch].isStepEnabled = false;
                continue;
            }

            uint8_t threshold = 100;
            switch (probType)
            {
            case 0x00:
                threshold = 100;
                break; // 100%
            case 0x10:
                threshold = 75;
                break; // 75%
            case 0x20:
                threshold = 50;
                break; // 50%
            case 0x30:
                threshold = 25;
                break; // 25%
            default:
                threshold = 100;
                break;
            }

            if ((rand() % 100) < threshold)
            {
                channels[ch].isStepEnabled = true;
            }
            else
            {
                channels[ch].isStepEnabled = false;
            }
        }
    }

    void processTickEvents(uint32_t now)
    {
        for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++)
        {
            if (channels[ch].requiredAllOn)
            {
                if (currentTick == 0)
                    fireTrigger(ch, now);
                continue;
            }

            if (channels[ch].requiredMute)
                continue;

            // 抽選ではずれていたら一切鳴らさない（ダブル/トリプレットもすべてOFF）
            if (!channels[ch].isStepEnabled)
                continue;

            uint8_t rawValue = patternMatrix[ch][currentStep];
            TriggerType type = (TriggerType)(rawValue & TYPE_MASK);
            bool fire = false;

            switch (type)
            {
            case TriggerType::ON:
                if (currentTick == 0)
                    fire = true;
                break;

            case TriggerType::SUBSTEP_D2:
                if (currentTick == 0 || currentTick == 3)
                    fire = true;
                break;

            case TriggerType::SUBSTEP_D3:
                if (currentTick == 0 || currentTick == 2 || currentTick == 4)
                    fire = true;
                break;

            case TriggerType::OFF:
            default:
                break;
            }

            if (fire)
            {
                fireTrigger(ch, now);
            }
        }
    }

    void fireTrigger(uint8_t ch, uint32_t now)
    {
        channels[ch].pulseOffTimeUs = now + triggerWidthUs;
        if (!channels[ch].outputState)
        {
            channels[ch].outputState = true;
            onOutputHigh(ch);
        }
    }

    void allOutputsLow()
    {
        for (uint8_t i = 0; i < NUM_CHANNELS; i++)
        {
            if (channels[i].outputState)
            {
                channels[i].outputState = false;
                onOutputLow(i);
            }
        }
    }
};