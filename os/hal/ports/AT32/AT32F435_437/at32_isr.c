/*
    ChibiOS - Copyright (C) 2023..2024 Zhaqian
    ChibiOS - Copyright (C) 2024 Maxjta

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
 * @file    AT32F435_437/at32_isr.c
 * @brief   AT32F435_437 ISR handler code.
 *
 * @addtogroup AT32F435xx_ISR
 * @{
 */

#include "hal.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

#define exint_serve_irq(intsts, channel) {                                       \
                                                                            \
  if ((intsts) & (1U << (channel))) {                                           \
    _pal_isr_code(channel);                                                 \
  }                                                                         \
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/
#include "at32_exint0.inc"
#include "at32_exint1.inc"
#include "at32_exint2.inc"
#include "at32_exint3.inc"
#include "at32_exint4.inc"
#include "at32_exint5_9.inc"
#include "at32_exint10_15.inc"
#include "at32_exint16.inc"
#include "at32_exint17.inc"
#include "at32_exint18.inc"
#include "at32_exint19.inc"
#include "at32_exint20.inc"
#include "at32_exint21.inc"
#include "at32_exint22.inc"

#include "at32_tmr1_9_10_11.inc"
#include "at32_tmr2.inc"
#include "at32_tmr3.inc"
#include "at32_tmr4.inc"
#include "at32_tmr5.inc"
#include "at32_tmr6.inc"
#include "at32_tmr7.inc"
#include "at32_tmr8_12_13_14.inc"
#include "at32_tmr20.inc"

#include "at32_usart1.inc"
#include "at32_usart2.inc"
#include "at32_usart3.inc"
#include "at32_uart4.inc"
#include "at32_uart5.inc"
#include "at32_usart6.inc"
#include "at32_uart7.inc"
#include "at32_uart8.inc"

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Enables IRQ sources.
 *
 * @notapi
 */
void irqInit(void) {

  exint0_irq_init();
  exint1_irq_init();
  exint2_irq_init();
  exint3_irq_init();
  exint4_irq_init();
  exint5_9_irq_init();
  exint10_15_irq_init();
  exint16_irq_init();
  exint17_irq_init();
  exint18_irq_init();
  exint19_irq_init();
  exint20_irq_init();
  exint21_irq_init();
  exint22_irq_init();

  tmr1_tmr9_tmr10_tmr11_irq_init();
  tmr2_irq_init();
  tmr3_irq_init();
  tmr4_irq_init();
  tmr5_irq_init();
  tmr6_irq_init();
  tmr7_irq_init();
  tmr8_tmr12_tmr13_tmr14_irq_init();
  tmr20_irq_init();

  usart1_irq_init();
  usart2_irq_init();
  usart3_irq_init();
  uart4_irq_init();
  uart5_irq_init();
  usart6_irq_init();
  uart7_irq_init();
  uart8_irq_init();
}

/**
 * @brief   Disables IRQ sources.
 *
 * @notapi
 */
void irqDeinit(void) {

  exint0_irq_deinit();
  exint1_irq_deinit();
  exint2_irq_deinit();
  exint3_irq_deinit();
  exint4_irq_deinit();
  exint5_9_irq_deinit();
  exint10_15_irq_deinit();
  exint16_irq_deinit();
  exint17_irq_deinit();
  exint18_irq_deinit();
  exint19_irq_deinit();
  exint20_irq_deinit();
  exint21_irq_deinit();
  exint22_irq_deinit();

  tmr1_tmr9_tmr10_tmr11_irq_deinit();
  tmr2_irq_deinit();
  tmr3_irq_deinit();
  tmr4_irq_deinit();
  tmr5_irq_deinit();
  tmr6_irq_deinit();
  tmr7_irq_deinit();
  tmr8_tmr12_tmr13_tmr14_irq_deinit();

  usart1_irq_deinit();
  usart2_irq_deinit();
  usart3_irq_deinit();
  uart4_irq_deinit();
  uart5_irq_deinit();
  usart6_irq_deinit();
  uart7_irq_deinit();
  uart8_irq_deinit();
}

/** @} */
