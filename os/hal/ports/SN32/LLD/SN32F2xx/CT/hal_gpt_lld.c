/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

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
 * @file    CT/hal_gpt_lld.c
 * @brief   SN32 GPT subsystem low level driver source.
 *
 * @addtogroup GPT
 * @{
 */

#include "hal.h"

#if HAL_USE_GPT || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/
#    define GPT_CLK SN32_HCLK
/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   GPTD1 driver identifier.
 * @note    The driver GPTD1 allocates the complex timer CT16B0 when enabled.
 */
#    if SN32_GPT_USE_CT16B0 || defined(__DOXYGEN__)
GPTDriver GPTD1;
#    endif

/**
 * @brief   GPTD2 driver identifier.
 * @note    The driver GPTD2 allocates the timer CT16B1 when enabled.
 */
#    if SN32_GPT_USE_CT16B1 || defined(__DOXYGEN__)
GPTDriver GPTD2;
#    endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#    if SN32_GPT_USE_CT16B0 || defined(__DOXYGEN__)
#        if !defined(SN32_CT16B0_SUPPRESS_ISR)
#            if !defined(SN32_CT16B0_HANDLER)
#                error "SN32_CT16B0_HANDLER not defined"
#            endif
/**
 * @brief   CT16B0 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SN32_CT16B0_HANDLER) {
    OSAL_IRQ_PROLOGUE();

    gpt_lld_serve_interrupt(&GPTD1);

    OSAL_IRQ_EPILOGUE();
}
#        endif /* !defined(SN32_CT16B0_SUPPRESS_ISR) */
#    endif     /* SN32_GPT_USE_CT16B0 */

#    if SN32_GPT_USE_CT16B1 || defined(__DOXYGEN__)
#        if !defined(SN32_CT16B1_SUPPRESS_ISR)
#            if !defined(SN32_CT16B1_HANDLER)
#                error "SN32_CT16B1_HANDLER not defined"
#            endif
/**
 * @brief   CT16B1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SN32_CT16B1_HANDLER) {
    OSAL_IRQ_PROLOGUE();

    gpt_lld_serve_interrupt(&GPTD2);

    OSAL_IRQ_EPILOGUE();
}
#        endif /* !defined(SN32_CT16B1_SUPPRESS_ISR) */
#    endif     /* SN32_GPT_USE_CT16B1 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level GPT driver initialization.
 *
 * @notapi
 */
void gpt_lld_init(void) {
#    if SN32_GPT_USE_CT16B0
    /* Driver initialization.*/
    gptObjectInit(&GPTD1);
#    endif

#    if SN32_GPT_USE_CT16B1
    /* Driver initialization.*/
    gptObjectInit(&GPTD2);
#    endif
}

/**
 * @brief   Configures and activates the GPT peripheral.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_start(GPTDriver *gptp) {
    uint16_t psc;

    if (gptp->state == GPT_STOP) {
        /* Clock activation.*/
#    if SN32_GPT_USE_CT16B0
        if (&GPTD1 == gptp) {
            sys1EnableCT16B0();
            CT16B0_ResetTimer();
#        if !defined(SN32_CT16B0_SUPPRESS_ISR)
            nvicEnableVector(SN32_CT16B0_NUMBER, SN32_GPT_CT16B0_IRQ_PRIORITY);
#        endif
            gptp->clock = GPT_CLK;
        }
#    endif

#    if SN32_GPT_USE_CT16B1
        if (&GPTD2 == gptp) {
            sys1EnableCT16B1();
            CT16B1_ResetTimer();
#        if !defined(SN32_CT16B1_SUPPRESS_ISR)
            nvicEnableVector(SN32_CT16B1_NUMBER, SN32_GPT_CT16B1_IRQ_PRIORITY);
#        endif
            gptp->clock = GPT_CLK;
        }
#    endif
    } else {
        /* Driver re-configuration scenario, it must be stopped first.*/
        SN32_CT_GPT_SET(gptp, config.TMRCTRL, CT16_CEN_DIS); /* Timer disabled.*/
#    if SN32_GPT_USE_CT16B0
        if (&GPTD1 == gptp) {
            CT16B0_ResetTimer(); /* Counter reset to zero.*/
        }
#    endif
#    if SN32_GPT_USE_CT16B1
        if (&GPTD2 == gptp) {
            CT16B1_ResetTimer(); /* Counter reset to zero.*/
        }
#    endif
    }

    /* Prescaler value calculation.*/
    psc = ((gptp->clock / gptp->config->frequency) - 1);
    osalDbgAssert((psc <= SN32_CT16_PRE_LIMIT) && /* Prescaler calculation.*/
                      ((psc + 1) * gptp->config->frequency) == gptp->clock,
                  "invalid frequency");

    /* Timer configuration.*/
    SN32_CT_GPT_SET(gptp, config.CNTCTRL, gptp->config->cntctrl);
    SN32_CT_GPT_SET(gptp, config.PRE, psc);                                  /* Prescaler value.*/
    SN32_CT_GPT_AND(gptp, irq.IC, mskCT_IC_Clear(SN32_CT16B1_MAX_CHANNELS)); /* Clear pending IRQs.*/
}

/**
 * @brief   Deactivates the GPT peripheral.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_stop(GPTDriver *gptp) {
    if (gptp->state == GPT_READY) {
        SN32_CT_GPT_SET(gptp, config.TMRCTRL, CT16_CEN_DIS);                     /* Timer disabled.*/
        SN32_CT_GPT_AND(gptp, irq.IC, mskCT_IC_Clear(SN32_CT16B1_MAX_CHANNELS)); /* Clear pending IRQs.*/

#    if SN32_GPT_USE_CT16B0
        if (&GPTD1 == gptp) {
#        if !defined(SN32_CT16B0_SUPPRESS_ISR)
            nvicDisableVector(SN32_CT16B0_NUMBER);
#        endif
            sys1DisableCT16B0();
        }
#    endif

#    if SN32_GPT_USE_CT16B1
        if (&GPTD2 == gptp) {
#        if !defined(SN32_CT16B1_SUPPRESS_ISR)
            nvicDisableVector(SN32_CT16B1_NUMBER);
#        endif
            sys1DisableCT16B1();
        }
#    endif
    }
}

/**
 * @brief   Starts the timer in continuous mode.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 * @param[in] interval  period in ticks
 *
 * @notapi
 */
void gpt_lld_start_timer(GPTDriver *gptp, gptcnt_t interval) {
#    if ((defined(SN32F280) || defined(SN32F290)) && defined(SN32_GPT_USE_CT16B0))
    SN32_CT_GPT_SET(gptp, MR[0], CT16_PWM_UNLOCK(interval - 1U)); /* Time constant.*/
#    else
    SN32_CT_GPT_SET(gptp, MR[0], (interval - 1U)); /* Time constant.*/
#    endif
#    if SN32_GPT_USE_CT16B0
    if (&GPTD1 == gptp) {
        CT16B0_ResetTimer(); /* Counter reset to zero.*/
    }
#    endif
#    if SN32_GPT_USE_CT16B1
    if (&GPTD2 == gptp) {
        CT16B1_ResetTimer(); /* Counter reset to zero.*/
    }
#    endif
    SN32_CT_GPT_AND(gptp, irq.IC, mskCT_IC_Clear(SN32_CT16B1_MAX_CHANNELS)); /* Clear pending IRQs.*/
    if (NULL != gptp->config->callback)
#    if ((defined(SN32F280) || defined(SN32F290)) && defined(SN32_GPT_USE_CT16B0))
        SN32_CT_GPT_OR(gptp, match.MCTRL, CT16_PWM_UNLOCK(mskCT16_MRnIE_EN(0)));
#    else
        SN32_CT_GPT_OR(gptp, match.MCTRL, mskCT16_MRnIE_EN(0));
#    endif

    SN32_CT_GPT_OR(gptp, config.TMRCTRL, mskCT16_CEN_EN);
}

/**
 * @brief   Stops the timer.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_stop_timer(GPTDriver *gptp) {
    SN32_CT_GPT_SET(gptp, config.TMRCTRL, CT16_CEN_DIS);                     /* Initially stopped.*/
    SN32_CT_GPT_AND(gptp, irq.IC, mskCT_IC_Clear(SN32_CT16B1_MAX_CHANNELS)); /* Clear pending IRQs.*/
#    if ((defined(SN32F280) || defined(SN32F290)) && defined(SN32_GPT_USE_CT16B0))
    SN32_CT_GPT_SET(gptp, match.MCTRL, (SN32_CT_GPT_GET(gptp, match.MCTRL) & ~mskCT16_MRnIE_EN(0))); /* Disable the interrupt    */
#    else
    SN32_CT_GPT_AND(gptp, match.MCTRL, ~mskCT16_MRnIE_EN(0)); /* Disable the interrupt*/
#    endif
}

/**
 * @brief   Starts the timer in one shot mode and waits for completion.
 * @details This function specifically polls the timer waiting for completion
 *          in order to not have extra delays caused by interrupt servicing,
 *          this function is only recommended for short delays.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 * @param[in] interval  time interval in ticks
 *
 * @notapi
 */
void gpt_lld_polled_delay(GPTDriver *gptp, gptcnt_t interval) {
#    if ((defined(SN32F280) || defined(SN32F290)))
#        if defined(SN32_GPT_USE_CT16B0)
    SN32_CT_GPT_SET(gptp, MR[0], CT16_PWM_UNLOCK(interval - 1U)); /* Time constant.*/
    SN32_CT_GPT_SET(gptp, match.MCTRL, CT16_PWM_UNLOCK(mskCT16_MRnIE_EN(0) | mskCT16_MRnSTOP_EN(0)));
#        else
    SN32_CT_GPT_SET(gptp, MR[0], (interval - 1U)); /* Time constant.*/
    SN32_CT_GPT_SET(gptp, match.MCTRL, (mskCT16_MRnIE_EN(0) | mskCT16_MRnSTOP_EN(0)));
#        endif
#    else
    SN32_CT_GPT_SET(gptp, MR[0], (interval - 1U)); /* Time constant.*/
    SN32_CT_GPT_SET(gptp, match.MCTRL, (mskCT16_MRnIE_EN(0) | mskCT16_MRnSTOP_EN(0)));
#    endif
    SN32_CT_GPT_AND(gptp, irq.IC, mskCT_IC_Clear(SN32_CT16B1_MAX_CHANNELS)); /* Clear pending IRQs.*/
    SN32_CT_GPT_OR(gptp, config.TMRCTRL, mskCT16_CEN_EN);
    while ((SN32_CT_GPT_GET(gptp, irq.RIS) & mskCT16_MRnIF(0)) != 0)
        ;
    SN32_CT_GPT_AND(gptp, irq.IC, mskCT_IC_Clear(SN32_CT16B1_MAX_CHANNELS)); /* Clear pending IRQs.*/
}

/**
 * @brief   Shared IRQ handler.
 *
 * @param[in] gptp      pointer to a @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_serve_interrupt(GPTDriver *gptp) {
    uint32_t ris;

    ris = SN32_CT_GPT_GET(gptp, irq.RIS);
    SN32_CT_GPT_SET(gptp, irq.IC, ris);
    if ((ris & mskCT16_MRnIF(0)) != 0) _gpt_isr_invoke_cb(gptp);
}

#endif /* HAL_USE_GPT */

/** @} */
