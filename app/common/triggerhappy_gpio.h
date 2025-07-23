/*!
 * TriggerHappy GPIOs
 * Copyright 2025 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>

#define PWM_INTR_PIN D25 // PMW4 B

#define EC1B D0
#define EC1A D2
#define BTN_MODE D6
#define BTN_A D3
#define BTN_B D1
#define BTN_RE D4
#define CLOCK A3
#define RESET A2
#define DATA A1
#define LED_R D8  //             PWM4 A
#define LED_G D9  //             PWM4 B
#define LED_B D7  //             PWM3 B
#define OUT1 D12  // SPI MISO(RX)PWM6 A
#define OUT2 D13  // SPI CS      PMW6 B
#define OUT3 D14  // SPI SCK     PMW7 A
#define OUT4 D15  // SPI MOSI(TX)PMW7 B
#define OUT5 D11  //             PWM5 B
#define OUT6 D10  //             PWM5 A
#define OUT_CV A0 //             PWM5 A
