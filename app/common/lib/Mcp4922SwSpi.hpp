/*!
 * MCP4922 software SPI class
 * Copyright 2025 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once

#include <Arduino.h>

class Mcp4922SwSpi
{
public:
    Mcp4922SwSpi()
        : _dataOutPin(255), _clockPin(255), _selectPin(255)
    {
    }

    void init(uint8_t dataOutPin, uint8_t clockPin, uint8_t selectPin)
    {
        _dataOutPin = dataOutPin;
        _clockPin = clockPin;
        _selectPin = selectPin;
        pinMode(_dataOutPin, OUTPUT);
        pinMode(_clockPin, OUTPUT);
        pinMode(_selectPin, OUTPUT);
        gpio_put(_dataOutPin, LOW);
        gpio_put(_clockPin, LOW);
        gpio_put(_selectPin, HIGH);
    }

    void out1(uint16_t value)
    {
        transfer(0x3000 | value);
    }

    void out2(uint16_t value)
    {
        transfer(0xB000 | value);
    }

private:
    uint8_t _dataOutPin;
    uint8_t _clockPin;
    uint8_t _selectPin;

    void transfer(uint16_t value)
    {
        gpio_put(_selectPin, LOW);
        transferCore((uint8_t)(value >> 8));
        transferCore((uint8_t)(value & 0xFF));
        gpio_put(_selectPin, HIGH);
    }

    void transferCore(uint8_t value)
    {
        for (uint8_t mask = 0x80; mask; mask >>= 1)
        {
            gpio_put(_dataOutPin, (value & mask) ? HIGH : LOW);
            gpio_put(_clockPin, HIGH);
            gpio_put(_clockPin, LOW);
        }
    }
};
