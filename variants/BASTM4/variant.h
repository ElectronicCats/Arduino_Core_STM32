/*
 *******************************************************************************
 * Copyright (c) 2018, STMicroelectronics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************
 */

#ifndef _VARIANT_ARDUINO_STM32_
#define _VARIANT_ARDUINO_STM32_

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

//
#define PA5  0  //SCK
#define PA6  1  //CIPO
#define PA7  2  //COPI
#define PB10 3  //SCL
#define PB11 4  //SDA
#define PA15 5  //D5 CS
#define PB3  6  //D6
#define PA0  7  //TX
#define PA1  8  //RX
#define PB4  9  //D9
#define PB6  10 //D10
#define PB7  11 //D11
#define PB8  12 //D12
#define PB9  13 //D13
#define PB13 14 //LED
#define PB2  15 //D2

// Analog pins
#define PB0  A0
#define PB1  A1
#define PA2  A2
#define PA3  A3
#define PA4  A4

#define NUM_DIGITAL_PINS        21
#define NUM_ANALOG_INPUTS       5
//#define NUM_ANALOG_FIRST        0 

// On-board LED pin number
#define LED_BUILTIN             14
#define LED_GREEN               LED_BUILTIN

// I2C Definitions
#define PIN_WIRE_SDA            4
#define PIN_WIRE_SCL            3

// SPI definitions
#define PIN_SPI_SS              5
#define PIN_SPI_MOSI            2
#define PIN_SPI_MISO            1
#define PIN_SPI_SCK             0

//ADC resolution is 12bits
#define ADC_RESOLUTION          12

//PWR resolution
//#define PWM_RESOLUTION          8
//#define PWM_FREQUENCY           1000
//#define PWM_MAX_DUTY_CYCLE      255

//Timer Definitions
#define TIMER_TONE              TIM6
#define TIMER_SERVO             TIM7

// UART Definitions
// Default pin used for 'Serial' instance (ex: ST-Link)
#define SERIAL_UART_INSTANCE    1
// Mandatory for Firmata
#define PIN_SERIAL_RX           8
#define PIN_SERIAL_TX           7

#ifdef __cplusplus
} // extern "C"
#endif
/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_MONITOR   Serial
//#define SERIAL_PORT_USBVIRTUAL Serial
#define SERIAL_PORT_HARDWARE  Serial1
#endif

#endif /* _VARIANT_ARDUINO_STM32_ */