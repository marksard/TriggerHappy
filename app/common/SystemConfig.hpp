/*!
 * SystemConfig
 * Copyright 2025 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once

#include <Arduino.h>

// 基本設定
struct SystemConfig
{
    char ver[15] = "TrigHappy_000\0";
    float vRef;
    float noiseFloor;

    SystemConfig()
    {
        vRef = 3.3f;
        noiseFloor = 0.0f;        
    }
};
