/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio
    ChibiOS - Copyright (C) 2023..2025 HorrorTroll
    ChibiOS - Copyright (C) 2023..2025 Zhaqian
    ChibiOS - Copyright (C) 2024..2025 Maxjta

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    USART/at32_usart6.inc
 * @brief   Shared USART6 handler.
 *
 * @addtogroup AT32_USART6_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Registry checks for robustness.*/
#if !defined(AT32_HAS_USART6)
#error "AT32_HAS_USART6 not defined in registry"
#endif

#if AT32_HAS_USART6

/* Priority settings checks.*/
#if !defined(AT32_IRQ_USART6_PRIORITY)
#error "AT32_IRQ_USART6_PRIORITY not defined in mcuconf.h"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(AT32_IRQ_USART6_PRIORITY)
#error "Invalid IRQ priority assigned to AT32_IRQ_USART6_PRIORITY"
#endif

#endif /* AT32_HAS_USART6 */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static inline void usart6_irq_init(void) {
#if defined(AT32_USART6_IS_USED)
  nvicEnableVector(AT32_USART6_NUMBER, AT32_IRQ_USART6_PRIORITY);
#endif
}

static inline void usart6_irq_deinit(void) {
#if defined(AT32_USART6_IS_USED)
  nvicDisableVector(AT32_USART6_NUMBER);
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if defined(AT32_USART6_IS_USED) || defined(__DOXYGEN__)
/**
 * @brief   USART6 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(AT32_USART6_HANDLER) {

  OSAL_IRQ_PROLOGUE();

#if HAL_USE_SERIAL
#if AT32_SERIAL_USE_USART6
  sd_lld_serve_interrupt(&SD6);
#endif
#endif

#if HAL_USE_SIO
#if AT32_SIO_USE_USART6
  sio_lld_serve_interrupt(&SIOD6);
#endif
#endif

#if HAL_USE_UART
#if AT32_UART_USE_USART6
  uart_lld_serve_interrupt(&UARTD6);
#endif
#endif

  OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/** @} */
