/*!
 * RGBLEDPWMControl class
 * Copyright 2025 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>

#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "MiniOsc.hpp"
#include "pwm_wrapper.h"

class RGBLEDPWMControl
{
public:
    enum class MenuColor
    {
        RED,
        GREEN,
        BLUE,
        YELLOW,
        CYAN,
        MAGENTA,
        WHITE,
        BLACK,
    };

    RGBLEDPWMControl()
        : _ledRPin(0), _ledGPin(0), _ledBPin(0),
          _ledRLevel(0), _ledGLevel(0), _ledBLevel(0),
          _ledRFreq(300), _ledGFreq(300), _ledBFreq(300),
          _color(MenuColor::BLACK),
          _ignoreMenuColor(false),
          _menuColorLevel(5) {}

    void init(float sampleFreq, uint16_t waveHeightBit, uint8_t ledRPin, uint8_t ledGPin, uint8_t ledBPin)
    {
        _ledRPin = ledRPin;
        _ledGPin = ledGPin;
        _ledBPin = ledBPin;

        _waveHeightBit = waveHeightBit;
        uint16_t pwmReso = 1 << waveHeightBit;

        initPWM(_ledRPin, pwmReso);
        _ledR.init(sampleFreq, waveHeightBit);
        _ledR.setWave(MiniOsc::Wave::SQU);
        _ledR.setFrequency(300);
        _ledR.setLevel(0);

        initPWM(_ledGPin, pwmReso);
        _ledG.init(sampleFreq, waveHeightBit);
        _ledG.setWave(MiniOsc::Wave::SQU);
        _ledG.setFrequency(300);
        _ledG.setLevel(0);

        initPWM(_ledBPin, pwmReso);
        _ledB.init(sampleFreq, waveHeightBit);
        _ledB.setWave(MiniOsc::Wave::SQU);
        _ledB.setFrequency(300);
        _ledB.setLevel(0);
    }

    void setMenuColor(MenuColor color)
    {
        _color = color;
    }

    void ignoreMenuColor(bool value)
    {
        _ignoreMenuColor = value;
    }

    void setWave(MiniOsc::Wave value)
    {
        _ledR.setWave(value);
        _ledG.setWave(value);
        _ledB.setWave(value);
    }

    void setFreq(float freq)
    {
        _ledRFreq = freq;
        _ledGFreq = freq;
        _ledBFreq = freq;
    }

    void resetFreq()
    {
        _ledRFreq = 300;
        _ledGFreq = 300;
        _ledBFreq = 300;
    }

    void setBlink()
    {
        _ledRFreq = 20;
        _ledGFreq = 20;
        _ledBFreq = 20;
    }

    void resetLevel()
    {
        _ledRLevel = 0;
        _ledGLevel = 0;
        _ledBLevel = 0;
    }

    void setRLevel(uint8_t level)
    {
        _ledRLevel = level;
    }

    void setGLevel(uint8_t level)
    {
        _ledGLevel = level;
    }

    void setBLevel(uint8_t level)
    {
        _ledBLevel = level;
    }

    void setLevel(uint8_t level)
    {
        _ledRLevel = level;
        _ledGLevel = level;
        _ledBLevel = level;
    }

    void setMenuColorLevel(int16_t level)
    {
        _menuColorLevel = level;
    }

    void setRLevelMap(int16_t level, int16_t minLevel, int16_t maxLevel)
    {
        level = constrain(map(level, minLevel, maxLevel, 0, _waveHeightBit), 0, _waveHeightBit);
        _ledRLevel = level;
    }

    void setGLevelMap(int16_t level, int16_t minLevel, int16_t maxLevel)
    {
        level = constrain(map(level, minLevel, maxLevel, 0, _waveHeightBit), 0, _waveHeightBit);
        _ledGLevel = level;
    }

    void setBLevelMap(int16_t level, int16_t minLevel, int16_t maxLevel)
    {
        level = constrain(map(level, minLevel, maxLevel, 0, _waveHeightBit), 0, _waveHeightBit);
        _ledBLevel = level;
    }

    void setLevelMap(int16_t level, int16_t minLevel, int16_t maxLevel, int16_t maxLedLevel)
    {
        level = constrain(map(level, minLevel, maxLevel, 0, maxLedLevel), 0, maxLedLevel);
        _ledRLevel = level;
        _ledGLevel = level;
        _ledBLevel = level;
    }

    void setRainbowLevel(int16_t level, int16_t minLevel, int16_t maxLevel)
    {
        level = constrain(map(level, minLevel, maxLevel, 0, 15), 0, 15);
        _ledRLevel = constrain(_rainbowR[level] - (11 - _waveHeightBit), 0, _waveHeightBit);
        _ledGLevel = constrain(_rainbowG[level] - (11 - _waveHeightBit), 0, _waveHeightBit);
        _ledBLevel = constrain(_rainbowB[level] - (11 - _waveHeightBit), 0, _waveHeightBit);
    }

    void process()
    {
        pwm_set_gpio_level(_ledRPin, _ledR.getWaveValue());
        pwm_set_gpio_level(_ledGPin, _ledG.getWaveValue());
        pwm_set_gpio_level(_ledBPin, _ledB.getWaveValue());
    }

    void update()
    {
        if (_ignoreMenuColor)
        {
            setLED(1, _ledRLevel, _ledRFreq);
            setLED(2, _ledGLevel, _ledGFreq);
            setLED(3, _ledBLevel, _ledBFreq);
            resetLevel();
            return;
        }

        uint8_t ledRMenuLevel = 0;
        uint8_t ledGMenuLevel = 0;
        uint8_t ledBMenuLevel = 0;

        switch (_color)
        {
        case MenuColor::RED:
            ledRMenuLevel = _menuColorLevel;
            ledGMenuLevel = 1;
            ledBMenuLevel = 1;
            break;
        case MenuColor::GREEN:
            ledRMenuLevel = 1;
            ledGMenuLevel = _menuColorLevel;
            ledBMenuLevel = 1;
            break;
        case MenuColor::BLUE:
            ledRMenuLevel = 1;
            ledGMenuLevel = 1;
            ledBMenuLevel = _menuColorLevel;
            break;
        case MenuColor::YELLOW:
            ledRMenuLevel = _menuColorLevel;
            ledGMenuLevel = _menuColorLevel;
            ledBMenuLevel = 1;
            break;
        case MenuColor::CYAN:
            ledRMenuLevel = 1;
            ledGMenuLevel = _menuColorLevel;
            ledBMenuLevel = _menuColorLevel;
            break;
        case MenuColor::MAGENTA:
            ledRMenuLevel = _menuColorLevel;
            ledGMenuLevel = 1;
            ledBMenuLevel = _menuColorLevel;
            break;
        case MenuColor::WHITE:
            ledRMenuLevel = _menuColorLevel;
            ledGMenuLevel = _menuColorLevel;
            ledBMenuLevel = _menuColorLevel;
            break;
        case MenuColor::BLACK:
        default:
            ledRMenuLevel = 0;
            ledGMenuLevel = 0;
            ledBMenuLevel = 0;
            break;
        }

        setLED(1, _ledRLevel + ledRMenuLevel, _ledRFreq);
        setLED(2, _ledGLevel + ledGMenuLevel, _ledGFreq);
        setLED(3, _ledBLevel + ledBMenuLevel, _ledBFreq);
        resetLevel();
    }

private:
    void setLED(uint8_t led, uint8_t level, float freq = 10)
    {
        if (led == 1)
        {
            _ledR.setFrequency(freq);
            _ledR.setLevel(level);
        }
        else if (led == 2)
        {
            _ledG.setFrequency(freq);
            _ledG.setLevel(level);
        }
        else if (led == 3)
        {
            _ledB.setFrequency(freq);
            _ledB.setLevel(level);
        }
    }

private:
    MiniOsc _ledR, _ledG, _ledB;
    uint8_t _ledRPin, _ledGPin, _ledBPin;
    uint8_t _ledRLevel, _ledGLevel, _ledBLevel;
    float _ledRFreq, _ledGFreq, _ledBFreq;
    MenuColor _color;
    bool _ignoreMenuColor;
    uint8_t _waveHeightBit;
    uint8_t _menuColorLevel;

    const int8_t _rainbowR[16] = {11, 11, 11,  8,  4,  0,  0,  0,  0,  0,  0,  0,  8, 11, 11, 11};
    const int8_t _rainbowG[16] = { 0,  0,  0,  0,  0,  0,  4,  8, 11, 11, 11, 11, 11, 11, 11, 11};
    const int8_t _rainbowB[16] = { 0,  8, 11, 11, 11, 11, 11, 11, 11,  8,  4,  0,  0,  0,  8, 11};
};