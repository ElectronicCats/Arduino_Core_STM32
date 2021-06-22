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
#if defined(ARDUINO_GENERIC_MP153AABX) || defined(ARDUINO_GENERIC_MP153AADX) ||\
    defined(ARDUINO_GENERIC_MP153CABX) || defined(ARDUINO_GENERIC_MP153CADX) ||\
    defined(ARDUINO_GENERIC_MP153DABX) || defined(ARDUINO_GENERIC_MP153DADX) ||\
    defined(ARDUINO_GENERIC_MP153FABX) || defined(ARDUINO_GENERIC_MP153FADX) ||\
    defined(ARDUINO_GENERIC_MP157AABX) || defined(ARDUINO_GENERIC_MP157AADX) ||\
    defined(ARDUINO_GENERIC_MP157CABX) || defined(ARDUINO_GENERIC_MP157CADX) ||\
    defined(ARDUINO_GENERIC_MP157DABX) || defined(ARDUINO_GENERIC_MP157DADX) ||\
    defined(ARDUINO_GENERIC_MP157FABX) || defined(ARDUINO_GENERIC_MP157FADX)
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
