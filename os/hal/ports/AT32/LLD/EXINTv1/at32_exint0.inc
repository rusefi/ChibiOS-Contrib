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
 * @file    EXINTv1/at32_exint0.inc
 * @brief   Shared EXINT0 handler.
 *
 * @addtogroup AT32_EXINT0_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Priority settings checks.*/
#if !defined(AT32_IRQ_EXINT0_PRIORITY)
#error "AT32_IRQ_EXINT0_PRIORITY not defined in mcuconf.h"
#endif

#if !defined(AT32_DISABLE_EXINT0_HANDLER) &&                                \
    !OSAL_IRQ_IS_VALID_PRIORITY(AT32_IRQ_EXINT0_PRIORITY)
#error "Invalid IRQ priority assigned to AT32_IRQ_EXINT0_PRIORITY"
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

static inline void exint0_irq_init(void) {
#if (HAL_USE_PAL && (PAL_USE_WAIT || PAL_USE_CALLBACKS))
  nvicEnableVector(AT32_EXINT0_NUMBER, AT32_IRQ_EXINT0_PRIORITY);
#endif
}

static inline void exint0_irq_deinit(void) {
#if (HAL_USE_PAL && (PAL_USE_WAIT || PAL_USE_CALLBACKS))
  nvicDisableVector(AT32_EXINT0_NUMBER);
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if (HAL_USE_PAL && (PAL_USE_WAIT || PAL_USE_CALLBACKS)) || defined(__DOXYGEN__)
#if !defined(AT32_DISABLE_EXINT0_HANDLER)
/**
 * @brief   EXINT[0] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(AT32_EXINT0_HANDLER) {
  uint32_t intsts;

  OSAL_IRQ_PROLOGUE();

  exintGetAndClearGroup1(1U << 0, intsts);

  exint_serve_irq(intsts, 0);

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/** @} */
