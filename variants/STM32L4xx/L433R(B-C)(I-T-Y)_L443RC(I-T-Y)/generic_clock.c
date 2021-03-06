/*
 *******************************************************************************
 * Copyright (c) 2020-2021, STMicroelectronics
 * All rights reserved.
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */
#if defined(ARDUINO_GENERIC_L433RBIX) || defined(ARDUINO_GENERIC_L433RBTX) ||\
    defined(ARDUINO_GENERIC_L433RBYX) || defined(ARDUINO_GENERIC_L433RCIX) ||\
    defined(ARDUINO_GENERIC_L433RCTX) || defined(ARDUINO_GENERIC_L433RCYX) ||\
    defined(ARDUINO_GENERIC_L443RCIX) || defined(ARDUINO_GENERIC_L443RCTX) ||\
    defined(ARDUINO_GENERIC_L443RCYX)
#include "pins_arduino.h"

/**
  * @brief  System Clock Configuration
  * @param  None
  * @retval None
  */
WEAK void SystemClock_Config(void)
{
  /* SystemClock_Config can be generated by STM32CubeMX */
#warning "SystemClock_Config() is empty. Default clock at reset is used."
}

#endif /* ARDUINO_GENERIC_* */
