/*
 *******************************************************************************
 * Copyright (c) 2021, STMicroelectronics
 * All rights reserved.
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */
#if defined(ARDUINO_NUCLEO_L496ZG_P)

#include "pins_arduino.h"

// Pin number
// Match Table 11. NUCLEO-L496ZG, NUCLEO-L496ZG-P pin assignments
// from UM2179 STM32 Nucleo-144 board
const PinName digitalPin[] = {
  PD_9,  //D0
  PD_8,  //D1
  PF_15, //D2
  PE_13, //D3
  PF_14, //D4
  PE_11, //D5
  PE_9,  //D6
  PF_13, //D7
  PF_12, //D8
  PD_15, //D9
  PD_14, //D10
  PA_7,  //D11
  PA_6,  //D12
  PA_5,  //D13
  PB_9,  //D14
  PB_8,  //D15
  PC_6,  //D16
  PB_15, //D17
  PB_13, //D18
  PB_12, //D19
  PA_4,  //D20
  PB_4,  //D21
  PB_5,  //D22
  PB_3,  //D23
  PA_4,  //D24
  PB_4,  //D25
  PA_2,  //D26
  PB_10, //D27
  PE_15, //D28
  PB_0,  //D29
  PE_12, //D30
  PE_14, //D31
  PA_0,  //D32
  PB_0,  //D33
  PE_0,  //D34
  NC,    //D35
  PB_10, //D36
  PE_15, //D37
  PE_14, //D38
  PE_12, //D39
  PE_10, //D40
  PE_7,  //D41
  PE_8,  //D42
  PC_8,  //D43
  PC_9,  //D44
  PC_10, //D45
  PC_11, //D46
  PC_12, //D47
  PD_2,  //D48
  PF_3,  //D49
  PF_5,  //D50
  PD_7,  //D51
  PD_6,  //D52
  PD_5,  //D53
  PD_4,  //D54
  PD_3,  //D55
  PE_2,  //D56
  PE_4,  //D57
  PE_5,  //D58
  PE_6,  //D59
  PE_3,  //D60
  PF_8,  //D61
  PF_7,  //D62
  PF_9,  //D63
  PG_1,  //D64
  PG_0,  //D65
  PD_1,  //D66
  PD_0,  //D67
  PF_0,  //D68
  PF_1,  //D69
  PF_2,  //D70
  PB_6,  //D71
  PB_2,  //D72
  // ST Morpho
  PA_8,  //D73
  PA_9,  //D74
  PA_10, //D75
  PA_11, //D76
  PA_12, //D77
  PA_15, //D78
  PB_7,  //D79 - LEDBLUE
  PB_14, //D80 - LEDRED
  PC_7,  //D81 - LEDGREEN
  PC_13, //D82 - USERBTN
  PC_14, //D83
  PC_15, //D84
  PD_10, //D85
  PD_11, //D86
  PD_12, //D87
  PD_13, //D88
  PE_1,  //D89
  PF_10, //D90
  PF_11, //D91
  PG_2,  //D92
  PG_3,  //D93
  PG_4,  //D94
  PG_5,  //D95
  PG_6,  //D96
  PG_7,  //D97 - Serial Tx
  PG_8,  //D98 - Serial Rx
  PG_9,  //D99
  PG_10, //D100
  PG_11, //D101
  PG_12, //D102
  PG_13, //D103
  PG_14, //D104
  PH_0,  //D105
  PH_1,  //D106
  // Analog pins
  PA_3,  //D107/A0
  PC_0,  //D108/A1
  PC_3,  //D109/A2
  PC_1,  //D110/A3
  PC_4,  //D111/A4
  PC_5,  //D112/A5
  PB_1,  //D113/A6
  PC_2,  //D114/A7
  PA_1,  //D115/A8
  PF_4,  //D116/A9
  PF_6,  //D117/A10
  PA_13, //D118
  PA_14  //D119
};

// Analog (Ax) pin number array
const uint32_t analogInputPin[] = {
  107, //A0
  108, //A1
  109, //A2
  110, //A3
  111, //A4
  112, //A5
  113, //A6
  114, //A7
  115, //A8
  116, //A9
  117, //A10
  11,  //A11
  12,  //A12
  13,  //A13
  20,  //A14
  26,  //A15
  29,  //A16
  32,  //A17
  49,  //A18
  50,  //A19
  61,  //A20
  62,  //A21
  63,  //A22
  90   //A23
};

// ----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  System Clock Configuration
  * @param  None
  * @retval None
  */
WEAK void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  /* Configure LSE Drive Capability */
  HAL_PWR_EnableBkUpAccess();

  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /* Initializes the CPU, AHB and APB busses clocks */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /* Initializes the CPU, AHB and APB busses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }

  /* Configure the main internal regulator output voltage */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
    Error_Handler();
  }

  /* Enable MSI Auto calibration */
  HAL_RCCEx_EnableMSIPLLMode();
}

#ifdef __cplusplus
}
#endif
#endif /* ARDUINO_NUCLEO_L496ZG_P */
