/*!
 * ClockDivider class
 * Copyright 2025 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>
#include "lib/TriggerOut.hpp"
#include "triggerhappy_gpio.h"
#include "triggerhappy_basicconfig.h"

// TriggerOutクラスのwritePinメソッドをオーバーライドして、ピンマスクを使用するクラス
class TriggerOutMaskPut : public TriggerOut {
public:
    TriggerOutMaskPut() : TriggerOut() {}
    TriggerOutMaskPut(uint8_t pin) : TriggerOut(pin) {};
    inline void writePin(int status) override {
        _pinMask = (status != 0) ? (1u << _pin) : 0;
    }
    inline uint32_t getPinMask() const { return _pinMask; }
private:
    uint32_t _pinMask; // ピンマスク
};

class TriggerOutManager {
public:
    inline void init() {
        _trigDurationMode = _trigDurationsSize - 1;
        initPins();
    }

    inline void initPins() {
        for (size_t i = 0; i < OUT_COUNT; ++i) {
            _triggerOuts[i].init(_outPins[i]);
        }
    }

    inline void reset() {
        for (size_t i = 0; i < OUT_COUNT; ++i) {
            _triggerOuts[i].set(0);
        }
    }

    inline void addDurationMode(int8_t encValue) {
        _trigDurationMode = constrain(_trigDurationMode + encValue, 0, _trigDurationsSize - 1);
    }

    // 一気に全てのトリガー出力を更新
    inline void process()
    {
        uint32_t mask = 0;
        for (size_t i = 0; i < OUT_COUNT; ++i) {
            // _triggerOuts[i].update(1);
            mask |= _triggerOuts[i].getPinMask();
        }
        gpio_put_masked(_outMask, mask);
    }

    inline uint8_t getDurationMode() const { return _trigDurationMode; }
    inline uint8_t getDuration() const { return _trigDurations[_trigDurationMode]; }
    inline size_t getDurationsSize() const { return _trigDurationsSize; }
    inline TriggerOutMaskPut* out(int index) { return &(_triggerOuts[index]); }
    inline bool isGateMode() const { return _trigDurationMode == _trigDurationsSize - 1; }

private:
    const uint8_t _outPins[OUT_COUNT] = {OUT1, OUT2, OUT3, OUT4, OUT5, OUT6};
    const int _outMask = (1u << OUT1) | (1u << OUT2) | (1u << OUT3) | (1u << OUT4) | (1u << OUT5) | (1u << OUT6);
    const uint8_t _trigDurations[7] = {2, 8, 16, 32, 64, 80, 100};
    static const size_t _trigDurationsSize = sizeof(_trigDurations) / sizeof(_trigDurations[0]);
    TriggerOutMaskPut _triggerOuts[OUT_COUNT];
    uint8_t _trigDurationMode;
};
