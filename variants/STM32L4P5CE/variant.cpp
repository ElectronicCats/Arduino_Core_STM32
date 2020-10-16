
 
  /*Copyright (c) 2017, STMicroelectronics
  All rights reserved.
 
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met
 
  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     andor other materials provided with the distribution.
  3. Neither the name of STMicroelectronics nor the names of its contributors
     may be used to endorse or promote products derived from this software
     without specific prior written permission.
 
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 

#include variant.h

#ifdef __cplusplus
extern C {
#endif

 Pin number
 This array allows to wrap Arduino pin number(Dx or x)
 to STM32 PinName (PYx)
const PinName digitalPin[] = {
PYx, Dx
  PB0, //D0- A0
  PB1, //D1- A1
  PA2, //D2- A2
  PA3, //D3- A3
  PA4, //D4- A4
  PA15, //D5
  PB3, //D6
  PA0, //D7 -TX
  PA1, //D8 -RX
  PB4, //D9
  PB6, //D10
  PB7, //D11
  PB8, //D12
  PB9, //D13
  PC13, //D14 -LED
  PB2, //D15 -D2
  PA5, //D16 -SCK
  PA6, //D17 -CIPO
  PA7, //D18 -COPI
  PB10, //D19 -SCL
  PB11, //D20 -SDA
};

#ifdef __cplusplus
}
#endif


  UART objects
 
// Replace PYx_Rx and PYx_Tx by UART_RX and UART_TX pin names
HardwareSerial  Serial(PYx_Rx, PYx_Tx);//  Connected to ST-Link

void serialEvent() __attribute__((weak));
void serialEvent() { }


 #ifdef ENABLE_SERIAL1
HardwareSerial  Serial1(PA1, PA0);

void serialEvent1() __attribute__((weak));
void serialEvent1() { }
#endif

void serialEventRun(void)
{
  if (Serial.available()) serialEvent();
#ifdef ENABLE_SERIAL1
  if (Serial1.available()) serialEvent1();
#endif
}

// ----------------------------------------------------------------------------

/*#ifdef __cplusplus
extern C {
#endif


   @brief  System Clock Configuration
   @param  None
   @retval None
  
WEAK void SystemClock_Config(void)
{
 /*Here copy the desired System Clock Configuration
 It could be generated thanks STM32CubeMX after code generation for ToolchainIDE 'SW4STM32',
 available in main.c)
 or
 copied from a STM32CubeYY project examples
 where 'YY' could be F0, F1, F2, F3, F4, F7, L0, L1, L4)*/
/*}

#ifdef __cplusplus
}
#endif*/