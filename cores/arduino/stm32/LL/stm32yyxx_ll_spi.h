#ifndef _STM32YYXX_LL_SPI_H_
#define _STM32YYXX_LL_SPI_H_
/* LL raised several warnings, ignore them */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wstrict-aliasing"

#ifdef STM32F0xx
  #include "stm32f0xx_ll_spi.h"
#elif STM32F1xx
  #include "stm32f1xx_ll_spi.h"
#elif STM32F2xx
  #include "stm32f2xx_ll_spi.h"
#elif STM32F3xx
  #include "stm32f3xx_ll_spi.h"
#elif STM32F4xx
  #include "stm32f4xx_ll_spi.h"
#elif STM32F7xx
  #include "stm32f7xx_ll_spi.h"
#elif STM32G0xx
  #include "stm32g0xx_ll_spi.h"
#elif STM32G4xx
  #include "stm32g4xx_ll_spi.h"
#elif STM32H7xx
  #include "stm32h7xx_ll_spi.h"
#elif STM32L0xx
  #include "stm32l0xx_ll_spi.h"
#elif STM32L1xx
  #include "stm32l1xx_ll_spi.h"
#elif STM32L4xx
  #include "stm32l4xx_ll_spi.h"
#elif STM32L5xx
  #include "stm32l5xx_ll_spi.h"
#elif STM32MP1xx
  #include "stm32mp1xx_ll_spi.h"
#elif STM32WBxx
  #include "stm32wbxx_ll_spi.h"
#endif
#elif STM32WLxx
  #include "stm32wlxx_ll_spi.h"
#pragma GCC diagnostic pop
#endif /* _STM32YYXX_LL_SPI_H_ */
