/*!
 * AnalogRead class
 * Copyright 2023 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once

#include <Arduino.h>
#include <hardware/adc.h>

/// @brief 12bitADC専用
class SmoothAnalogRead
{
public:
    SmoothAnalogRead()
    {
        _rawValueIndex = 0;
        _rawValuesSum = 0;
        for (uint8_t i = 0; i < averageCount; ++i)
        {
            _rawValues[i] = 0;
        }
    }

    SmoothAnalogRead(byte pin) : SmoothAnalogRead()
    {
        init(pin);
    }

    /// @brief ピン設定
    /// @param pin
    void init(byte pin)
    {
        _pin = pin;
        _value = 0;
        _valueOld = 65535;
        if (_adc_initted == false)
        {
            analogReadResolution(12);
            adc_init();
            _adc_initted = true;
        }
        adc_gpio_init(pin);
        // pinMode(pin, INPUT);
    }

    /// @brief adc値を直接取得
    /// @return
    uint16_t analogReadDirectFast()
    {
        _valueOld = _value;
        _value = readPinFast();
        return _value;
    }

    /// @brief 両端補正と4bitを落としたadc値の取得
    /// @return
    uint16_t analogReadDropLow4bit()
    {
        _valueOld = _value;
        uint16_t value = readPinFast();
        if (value < 0x16)
        {
            _value = 0;
        }
        else if (value > 0xFFB)
        {
            _value = 0xFFF;
        }
        else
        {
            _value = value & 0xFFF0;
        }
        return _value;
    }

    /// @brief adc値の移動平均値、ローパスフィルタを通した値を取得
    /// @return
    uint16_t analogRead(bool smooth = true)
    {
        _valueOld = _value;
        int16_t aval = analogReadAverage();
        if (smooth)
        {
            _value = (_value * 0.95) + (aval * 0.05024);
            return _value;
        }
        _value = aval;
        return _value;
    }

    uint16_t getValue()
    {
        return _value;
    }

    bool hasChanged()
    {
        return _valueOld != _value;
    }

protected:
    // 平均カウントは4回（>>2で割算） + 1（平均から除外する最古のデータ）を記憶
    static constexpr uint8_t averageShiftCount = 2;
    static constexpr uint32_t averageCount = (1 << averageShiftCount) + 1;
    static bool _adc_initted;
    byte _pin;
    uint16_t _value;
    uint16_t _valueOld;
    uint8_t _rawValueIndex;
    uint16_t _rawValues[averageCount];
    int32_t _rawValuesSum;

    /// @brief ピン値読込
    /// @return
    /// @note 必要最低限の処理のため、これを利用する場合すべてのADCは同一スレッドで読むこと
    virtual uint16_t readPinFast()
    {
        adc_select_input(_pin - A0);
        return adc_read();
    }

    /// @brief ピン値読込
    /// @return
    virtual uint16_t readPin()
    {
        return ::analogRead(_pin);
    }

    /// @brief adcの移動平均を取得
    /// @return
    uint16_t analogReadAverage()
    {
        uint16_t value = readPinFast();
        _rawValues[_rawValueIndex] = value;
        _rawValueIndex++;
        if (_rawValueIndex >= averageCount)
        {
            _rawValueIndex = 0;
        }

        // 積算値から最古のデータを引き、最新のデータを足すことでループなしで移動平均を出す
        uint16_t lastValue = _rawValues[_rawValueIndex];
        _rawValuesSum = _rawValuesSum - lastValue + value;
        return (_rawValuesSum >> averageShiftCount);
    }
};

bool SmoothAnalogRead::_adc_initted = false;
