/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio
    ChibiOS - Copyright (C) 2023..2025 HorrorTroll
    ChibiOS - Copyright (C) 2023..2025 Zhaqian

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
 * @file    EXINTv1/at32_exint20.inc
 * @brief   Shared EXINT20 handler.
 *
 * @addtogroup AT32_EXINT20_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Priority settings checks.*/
#if !defined(AT32_IRQ_EXINT20_PRIORITY)
#error "AT32_IRQ_EXINT20_PRIORITY not defined in mcuconf.h"
#endif

#if !defined(AT32_DISABLE_EXINT20_HANDLER) &&                               \
    !OSAL_IRQ_IS_VALID_PRIORITY(AT32_IRQ_EXINT20_PRIORITY)
#error "Invalid IRQ priority assigned to AT32_IRQ_EXINT20_PRIORITY"
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static inline void exint20_irq_init(void) {
#if defined(AT32_EXINT20_IS_USED)
  nvicEnableVector(AT32_EXINT20_NUMBER, AT32_IRQ_EXINT20_PRIORITY);
#endif
}

static inline void exint20_irq_deinit(void) {
#if defined(AT32_EXINT20_IS_USED)
  nvicDisableVector(AT32_EXINT20_NUMBER);
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if defined(AT32_EXINT20_IS_USED) || defined(__DOXYGEN__)
#if !defined(AT32_DISABLE_EXINT20_HANDLER)
/**
 * @brief   EXINT[20] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(AT32_EXINT20_HANDLER) {
  uint32_t intsts;

  OSAL_IRQ_PROLOGUE();

  exintGetAndClearGroup1(1U << 20, intsts);

  /* Could be unused.*/
  (void)intsts;

#if defined(AT32_EXINT20_ISR)
  AT32_EXINT20_ISR(intsts, 20);
#endif

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/** @} */
