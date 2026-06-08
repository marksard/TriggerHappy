/*!
 * ClockDividerMultiplier class
 * Copyright 2026 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once

#include <stdint.h>
#include "pico/stdlib.h"

/// @brief Clock Divider / Multiplier
/// @details
/// 入力クロックを基準に、各チャンネルごとに
/// Divide(分周)またはMultiply(逓倍)されたクロックを生成する。
///
/// Multiply動作はDDS(Direct Digital Synthesis)方式を使用する。
///
/// 特徴:
/// - チャンネルごとに独立したDivide / Multiply設定
/// - チャンネルごとに独立した倍率設定
/// - Trigger出力または50% Gate出力
/// - RESET入力対応
/// - CLOCK入力停止時の自動タイムアウト停止
///
/// RESET入力を受けると、次回CLOCK立ち上がり時に
/// 全チャンネルの位相をリセットする。
class ClockDividerMultiplier
{
public:
    static constexpr uint8_t NUM_CHANNELS = 3;
    static constexpr uint64_t PHASE_SCALE = 0x100000000ULL;

    enum class PulseMode : uint8_t
    {
        TRIGGER,
        GATE_50
    };

    enum class RatioIndex : uint8_t
    {
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
        MUL5,
        MUL6,
        MUL7,
        MUL8,
        MUL12,
        MUL16,

        COUNT
    };

    /// @brief チャンネル設定
    struct Channel
    {
        struct RatioInfo
        {
            bool multiply;
            uint8_t factor;
        };

        static constexpr RatioInfo RATIO_TABLE[] =
            {
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
                {true, 4},
                {true, 5},
                {true, 6},
                {true, 7},
                {true, 8},
                {true, 12},
                {true, 16}};

        bool multiply = false;
        uint8_t factor = 2;

        PulseMode pulseMode = PulseMode::GATE_50;

        uint32_t phase = 0;
        uint32_t lastUpdateUs = 0;

        bool outputState = false;
        uint32_t pulseOffTimeUs = 0;

        RatioIndex ratioIndex = RatioIndex::MUL1;

        void addRatio(int8_t delta)
        {
            int32_t r = constrain((int8_t)ratioIndex + delta, 0, (int8_t)RatioIndex::COUNT - 1);
            setRatio((RatioIndex)r);
        }

        void addPulseMode(int8_t delta)
        {
            pulseMode = (PulseMode)constrain((int8_t)pulseMode + delta, (int8_t)PulseMode::TRIGGER, (int8_t)PulseMode::GATE_50);
        }

        void setRatio(RatioIndex r)
        {
            ratioIndex = r;
            multiply = RATIO_TABLE[(int8_t)r].multiply;
            factor = RATIO_TABLE[(int8_t)r].factor;
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

    /// @brief Triggerモード時のパルス幅設定
    /// @param us パルス幅[us]
    void setTriggerWidthUs(uint32_t us)
    {
        triggerWidthUs = us;
    }

    /// @brief RESET入力立ち上がり通知
    /// @details
    /// 実際のリセットは次回CLOCK入力時に行われる。
    void onResetRise()
    {
        resetPending = true;
    }

    /// @brief CLOCK入力立ち上がり通知
    /// @details
    /// クロック周期測定とDivide/Multiplyイベント生成を行う。
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

    /// @brief メイン更新処理
    /// @details
    /// メインループから周期的に呼び出す。
    ///
    /// 以下を処理する:
    /// - CLOCKタイムアウト監視
    /// - パルスOFF処理
    /// - Multiply用DDS更新
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
    /// @brief 出力をHIGHにする
    /// @param ch チャンネル番号
    /// @details
    /// 継承先でgpio_put()などを実装する。
    virtual void onOutputHigh(uint8_t ch)
    {
    }

    /// @brief 出力をLOWにする
    /// @param ch チャンネル番号
    /// @details
    /// 継承先でgpio_put()などを実装する。
    virtual void onOutputLow(uint8_t ch)
    {
    }

private:
    uint32_t triggerWidthUs = 5000;
    uint32_t clockPeriodUs = 500000;
    uint32_t lastClockUs = 0;

    bool resetPending = false;
    bool clockRunning = false;

    /// @brief 出力イベント発生
    /// @param ch チャンネル番号
    /// @param now 現在時刻[us]
    /// @param intervalUs 出力周期[us]
    /// @details
    /// PulseModeに応じたパルス生成を行う。
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