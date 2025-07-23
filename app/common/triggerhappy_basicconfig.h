/*!
 * Basic config
 * Copyright 2025 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>

// 基本使用設定
#define CPU_CLOCK 133000000.0
#define INTR_PWM_RESO 512
#define PWM_BIT 11
#define PWM_RESO 2048
#define ADC_BIT 12
#define ADC_RESO 4096
#define DAC_MAX_MILLVOLT 5000 // mV
#define SAMPLE_FREQ ((CPU_CLOCK / INTR_PWM_RESO) / 8) // 32470.703125khz

// 出力数
#define OUT_COUNT 6

// TriggerHappy 11bit PWM DC出力誤差補正 LUT（スプライン補完, int16_t）
const int16_t PWMCVDCOutputErrorLUT[61] = {
 1, 1, 0, 0, 0, 0, 0,-1, 0,-2,
-2,-2,-3,-3,-3,-3,-3,-3,-5,-5,
-5,-5,-6,-5,-5,-5,-6,-6,-6,-6,
-5,-5,-5,-5,-5,-6,-6,-6,-5,-4,
-4,-4,-4,-5,-5,-3,-3,-3,-3,-2,
-1,-2,-2, 0,-1, 0, 0, 0, 0, 0,
 0
};
