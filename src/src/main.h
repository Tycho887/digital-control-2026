/***************************************************************************
 *   Copyright (C) 2024 by DTU
 *   jcan@dtu.dk
 *
 *   Main hardware configuration (pin usage)
 *
 * The MIT License (MIT)  https://mit-license.org/
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the “Software”), to deal in the Software without restriction, 
 * including without limitation the rights to use, copy, modify, merge, publish, distribute, 
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software 
 * is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies 
 * or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
 * THE SOFTWARE. */

#ifndef REGBOT_MAIN_H
#define REGBOT_MAIN_H


#define MISSING_SYSCALL_NAMES

#include <core_pins.h>

#define REGBOT_HW41

#define PIN_LED_DEBUG           13 // (LED_BUILTIN)
#define PIN_LED_STATUS          0
#define PIN_START_BUTTON        37
#define PIN_LINE_LED_HIGH       34
#define PIN_LINE_LED_LOW        33
// #define PIN_DISABLE2            51 // (not used)
#define PIN_POWER_IR            36
#define PIN_POWER_ROBOT         1
#define PIN_SUPPLY_CURRENT      A14
#define PIN_IR_RAW_1            A15
#define PIN_IR_RAW_2            A16
#define PIN_BATTERY_VOLTAGE     A17
#define PIN_LEFT_MOTOR_CURRENT  A0
#define PIN_RIGHT_MOTOR_CURRENT A1
#define PIN_LINE_SENSOR_0       A6
#define PIN_LINE_SENSOR_1       A13
#define PIN_LINE_SENSOR_2       A7
#define PIN_LINE_SENSOR_3       A12
#define PIN_LINE_SENSOR_4       A8
#define PIN_LINE_SENSOR_5       A11
#define PIN_LINE_SENSOR_6       A9
#define PIN_LINE_SENSOR_7       A10

// Motor Controller pins
#define PIN_LEFT_PWM            2
#define PIN_LEFT_DIR            3
#define PIN_RIGHT_PWM           4
#define PIN_RIGHT_DIR           5
// #define PIN_LEFT_ENCODER_A      29
// #define PIN_LEFT_ENCODER_B      28
// #define PIN_RIGHT_ENCODER_A     31
// #define PIN_RIGHT_ENCODER_B     30
#define PIN_LEFT_FAULT          38
#define PIN_RIGHT_FAULT         32
#define PIN_MOTORS_ENABLE         32
#define M1ENC_A         29
#define M1ENC_B         28
#define M2ENC_A         31
#define M2ENC_B         30
// #define M1ENC_A         PIN_LEFT_ENCODER_A
// #define M1ENC_B         PIN_LEFT_ENCODER_B
// #define M2ENC_A         PIN_RIGHT_ENCODER_A
// #define M2ENC_B         PIN_RIGHT_ENCODER_B


#endif
