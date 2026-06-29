/*!
 * Button class
 * Copyright 2023 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once

#include <Arduino.h>

class Button
{
public:
    enum
    {
        BUTTON_NONE         = 0,
        BUTTON_DOWN         = 1,
        BUTTON_UP           = 2,
        BUTTON_HOLDING      = 3,
        BUTTON_HOLDED       = 4,

        BUTTON_EDGE_FLAG    = 0x08,

        BUTTON_HOLDING_EDGE = BUTTON_HOLDING | BUTTON_EDGE_FLAG
    };

public:
    Button() {}

    Button(uint8_t pin)
    {
        init(pin);
    }

    /// @brief ピン設定
    void init(uint8_t pin, bool needWait = true, bool pullup = true, bool invert = false)
    {
        _pin = pin;
        _pinState = 0;
        _holdStage = 0;
        _holdTime = 500 * 1000;
        _lastMicros = 0;
        _leadLastMicros = 0;
        _lastResult = 0;
        _invert = invert;

        pinMode(pin, pullup ? INPUT_PULLUP : INPUT);

        // 空読み
        for (int i = 0; i < 8; ++i)
        {
            uint8_t value = readPin();
            _pinState = (_pinState << 1) | value;
        }

        _needWait = needWait;
    }

    void init(uint8_t pin, PinMode mode, bool needWait = true, bool invert = false)
    {
        _pin = pin;
        _pinState = 0;
        _holdStage = 0;
        _holdTime = 500 * 1000;
        _lastMicros = 0;
        _leadLastMicros = 0;
        _lastResult = 0;
        _invert = invert;
        _needWait = needWait;

        pinMode(pin, mode);

        // 空読み
        for (int i = 0; i < 8; ++i)
        {
            uint8_t value = readPin();
            _pinState = (_pinState << 1) | value;
        }

    }

    /// @brief ボタン状態を取得
    /// @return
    /// 0 : None
    /// 1 : Button down
    /// 2 : Button up
    /// 3 : Holding
    /// 4 : Holded
    /// 11: Holding edge (3 | 0x08)
    inline uint8_t getState()
    {
        uint8_t value = readPin();

        if (_needWait && (micros() - _leadLastMicros) < 1000)
        {
            return _lastResult;
        }

        _leadLastMicros = micros();
        _lastResult = BUTTON_NONE;

        // 簡単チャタ取り
        _pinState = (_pinState << 1) | value;

        // Holding state
        if (_holdStage == 2)
        {
            // Holded
            if (_pinState == 0x0F)
            {
                _holdStage = 0;
                _lastResult = BUTTON_HOLDED;
            }
            // Holding continue
            else if (_pinState == 0x00)
            {
                _lastResult = BUTTON_HOLDING;
            }

            return _lastResult;
        }

        // Button down
        if (_pinState == 0xF0)
        {
            _lastResult = BUTTON_DOWN;
            _holdStage = 0;
        }
        // Button up
        else if (_pinState == 0x0F)
        {
            _lastResult = BUTTON_UP;
            _holdStage = 0;
        }
        // Hold check
        else if (_pinState == 0x00)
        {
            // Start hold check
            if (_holdStage == 0)
            {
                _holdStage = 1;
                _lastMicros = micros();
            }
            // Hold confirm
            else if (_holdStage == 1 &&
                     (micros() - _lastMicros) >= _holdTime)
            {
                _holdStage = 2;
                _lastResult = BUTTON_HOLDING_EDGE;
            }
        }

        return _lastResult;
    }

    void setHoldTime(int16_t mills)
    {
        _holdTime = (ulong)mills * 1000;
    }

    uint8_t getValue()
    {
        return _lastResult;
    }

protected:
    uint8_t _pin;
    uint8_t _pinState;
    uint8_t _holdStage;

    ulong _lastMicros;
    ulong _holdTime;
    ulong _leadLastMicros;

    uint8_t _lastResult;

    bool _needWait;
    bool _invert;

    /// @brief ピン値読込
    virtual uint8_t readPin()
    {
        bool result = gpio_get(_pin);
        return _invert ? !result : result;
    }
};
