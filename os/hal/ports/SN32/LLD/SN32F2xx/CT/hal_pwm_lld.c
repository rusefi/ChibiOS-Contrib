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
 * @file    CT/hal_pwm_lld.c
 * @brief   SN32 PWM subsystem low level driver header.
 *
 * @addtogroup PWM
 * @{
 */

#include "hal.h"

#if HAL_USE_PWM || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/
#define PWM_CLK                          SN32_HCLK
/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   PWMD1 driver identifier.
 * @note    The driver PWMD1 allocates the complex timer CT16B1 when enabled.
 */
#if SN32_PWM_USE_CT16B1 || defined(__DOXYGEN__)
PWMDriver PWMD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if SN32_PWM_USE_CT16B1 || defined(__DOXYGEN__)
#if !defined(SN32_CT16B1_SUPPRESS_ISR)
#if !defined(SN32_CT16B1_HANDLER)
#error "SN32_CT16B1_HANDLER not defined"
#endif
/**
 * @brief   CT16B1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SN32_CT16B1_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  pwm_lld_serve_interrupt(&PWMD1);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(SN32_CT16B0_SUPPRESS_ISR) */
#endif /* SN32_PWM_USE_CT16B1 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level PWM driver initialization.
 *
 * @notapi
 */
void pwm_lld_init(void) {

#if SN32_PWM_USE_CT16B1
  /* Driver initialization.*/
  pwmObjectInit(&PWMD1);
  PWMD1.channels = PWM_CHANNELS;
  PWMD1.ct = SN32_CT16B1;
#endif
}

/**
 * @brief   Configures and activates the PWM peripheral.
 * @note    Starting a driver that is already in the @p PWM_READY state
 *          disables all the active channels.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_start(PWMDriver *pwmp) {
  uint32_t psc;
  uint32_t pwmctrl;
  uint32_t pwmctrl2;
  uint32_t pwmen;
  uint32_t pwmioen;

  if (pwmp->state == PWM_STOP) {
    /* Clock activation and timer reset.*/
#if SN32_PWM_USE_CT16B1
    if (&PWMD1 == pwmp) {
      sys1EnableCT16B1();
      CT16B1_ResetTimer();
#if !defined(SN32_CT16B1_SUPPRESS_ISR)
      nvicEnableVector(SN32_CT16B1_NUMBER, SN32_PWM_CT16B1_IRQ_PRIORITY);
#endif
    pwmp->clock = PWM_CLK;
    }
#endif

#if (defined(SN32F240B)|| defined(SN32F240C))
  /* PFPA - Map all PWM outputs to their PWM A pins */
  SN_PFPA->CT16B1 = 0x00000000;
  /* PFPA assignment for PWM B-pin mapping.*/
  for(uint8_t i=0; i<PWM_CHANNELS; i++){
    if(pwmp->config->channels[i].pfpamsk != 0) {
      SN_PFPA->CT16B1 |= (1<<i);
    }
  }
#endif

  /* Channel PWM mode selection and polarities setup.*/
  pwmctrl = 0;
  pwmctrl2 = 0;
  pwmen = 0;
  pwmioen = 0;
  volatile uint32_t *pwmctrl_registers[] = {&pwmctrl, &pwmctrl2};
  for(uint8_t i=0; i < PWM_CHANNELS; i++) {
    switch (pwmp->config->channels[i].mode & PWM_OUTPUT_MASK) {
      case PWM_OUTPUT_ACTIVE_LOW:
        *pwmctrl_registers[(i > 15) ? 1 : 0] |= mskCT16_PWMnMODE_1(i);
        pwmen |= mskCT16_PWMnEN_EN(i);
        pwmioen |= mskCT16_PWMnIOEN_EN(i);
        break;
      case PWM_OUTPUT_ACTIVE_HIGH:
        *pwmctrl_registers[(i > 15) ? 1 : 0] |= mskCT16_PWMnMODE_2(i);
        pwmen |= mskCT16_PWMnEN_EN(i);
        pwmioen |= mskCT16_PWMnIOEN_EN(i);
        break;
    }
  }
  pwmp->ct->PWMCTRL  = pwmctrl;
  pwmp->ct->PWMCTRL2 = pwmctrl2;
  pwmp->ct->PWMENB   = pwmen;
  pwmp->ct->PWMIOENB = pwmioen;
  }
  else {
    /* Driver re-configuration scenario, it must be stopped first.*/
    pwmp->ct->TMRCTRL = CT16_CEN_DIS;       /* Timer disabled.              */
                                            /* Counter reset to zero.       */
    pwmp->ct->TMRCTRL = (mskCT16_CRST); //Set CT16B1 as the up-counting mode.    
    while (pwmp->ct->TMRCTRL & mskCT16_CRST); // Wait until timer reset done.
  }

  /* Timer configuration.*/
  psc = (pwmp->clock / pwmp->config->frequency) - 1;
  osalDbgAssert((psc <= 0xFF) &&     /* Prescaler calculation.             */
                ((psc + 1) * pwmp->config->frequency) == pwmp->clock,
                "invalid frequency");
  pwmp->ct->PRE  = psc;
  pwmp->ct->MR[PWM_CHANNELS] = pwmp->period - 1;

#if SN32_PWM_USE_ONESHOT || defined(__DOXYGEN__)
  volatile uint32_t *mctrl_registers[] = {&(pwmp->ct->MCTRL), &(pwmp->ct->MCTRL2), &(pwmp->ct->MCTRL3)};
  // Determine which MCTRL register to use
  *mctrl_registers[MCTRL_INDEX] |= mskCT16_MRnSTOP_EN(PWM_CHANNELS);
#elif !defined(SN32_PWM_NO_RESET)
  volatile uint32_t *mctrl_registers[] = {&(pwmp->ct->MCTRL), &(pwmp->ct->MCTRL2), &(pwmp->ct->MCTRL3)};
  // Determine which MCTRL register to use
  *mctrl_registers[MCTRL_INDEX] |= mskCT16_MRnRST_EN(PWM_CHANNELS);
#endif
  pwmp->ct->IC       &= 0x1FFFFFF;           /* Clear pending IRQs.          */

  /* Timer configured and started.*/
  pwmp->ct->TMRCTRL |= mskCT16_CEN_EN;
}

/**
 * @brief   Deactivates the PWM peripheral.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_stop(PWMDriver *pwmp) {

  /* If in ready state then disables the PWM clock.*/
  if (pwmp->state == PWM_READY) {
    pwmp->ct->TMRCTRL = CT16_CEN_DIS;       /* Timer disabled.              */
    pwmp->ct->IC &= 0x1FFFFFF;              /* Clear pending IRQs.          */

#if SN32_PWM_USE_CT16B1
    if (&PWMD1 == pwmp) {
#if !defined(SN32_CT16B1_SUPPRESS_ISR)
      nvicDisableVector(SN32_CT16B1_NUMBER);
#endif
      sys1DisableCT16B1();
    }
#endif
  }
}

/**
 * @brief   Enables a PWM channel.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @post    The channel is active using the specified configuration.
 * @note    The function has effect at the next cycle start.
 * @note    Channel notification is not enabled.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...channels-1)
 * @param[in] width     PWM pulse width as clock pulses number
 *
 * @notapi
 */
void pwm_lld_enable_channel(PWMDriver *pwmp,
                            pwmchannel_t channel,
                            pwmcnt_t width) {
  if(channel < PWM_CHANNELS) {
    /* Changing channel duty cycle on the fly.*/
    pwmp->ct->MR[channel] = width;
    pwmp->ct->PWMIOENB |= mskCT16_PWMnIOEN_EN(channel);
  }
}

/**
 * @brief   Disables a PWM channel and its notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @post    The channel is disabled and its output line returned to the
 *          idle state.
 * @note    The function has effect at the next cycle start.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...channels-1)
 *
 * @notapi
 */
void pwm_lld_disable_channel(PWMDriver *pwmp, pwmchannel_t channel) {
  if(channel < PWM_CHANNELS) {
    pwmp->ct->IC |= mskCT16_MRnIC(channel);
    pwmp->ct->PWMIOENB &= ~mskCT16_PWMnIOEN_EN(channel);
  }
}

/**
 * @brief   Enables the periodic activation edge notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @note    If the notification is already enabled then the call has no effect.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_enable_periodic_notification(PWMDriver *pwmp) {
  volatile uint32_t *mctrl_registers[] = {&(pwmp->ct->MCTRL), &(pwmp->ct->MCTRL2), &(pwmp->ct->MCTRL3)};
  // Determine which MCTRL register to use
  *mctrl_registers[MCTRL_INDEX] |= mskCT16_MRnIE_EN(PWM_CHANNELS);
}

/**
 * @brief   Disables the periodic activation edge notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @note    If the notification is already disabled then the call has no effect.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_disable_periodic_notification(PWMDriver *pwmp) {
  volatile uint32_t *mctrl_registers[] = {&(pwmp->ct->MCTRL), &(pwmp->ct->MCTRL2), &(pwmp->ct->MCTRL3)};
  // Determine which MCTRL register to use
  *mctrl_registers[MCTRL_INDEX] &= ~mskCT16_MRnIE_EN(PWM_CHANNELS);
}

/**
 * @brief   Enables a channel de-activation edge notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @pre     The channel must have been activated using @p pwmEnableChannel().
 * @note    If the notification is already enabled then the call has no effect.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...channels-1)
 *
 * @notapi
 */
void pwm_lld_enable_channel_notification(PWMDriver *pwmp,
                                         pwmchannel_t channel) {
  volatile uint32_t *mctrl_registers[] = {&(pwmp->ct->MCTRL), &(pwmp->ct->MCTRL2), &(pwmp->ct->MCTRL3)};
  // Determine which MCTRL register to use
  *mctrl_registers[MCTRL_INDEX] |= mskCT16_MRnIE_EN(channel);
}

/**
 * @brief   Disables a channel de-activation edge notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @pre     The channel must have been activated using @p pwmEnableChannel().
 * @note    If the notification is already disabled then the call has no effect.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...channels-1)
 *
 * @notapi
 */
void pwm_lld_disable_channel_notification(PWMDriver *pwmp,
                                          pwmchannel_t channel) {
  volatile uint32_t *mctrl_registers[] = {&(pwmp->ct->MCTRL), &(pwmp->ct->MCTRL2), &(pwmp->ct->MCTRL3)};
  // Determine which MCTRL register to use
  *mctrl_registers[MCTRL_INDEX] &= ~mskCT16_MRnIE_EN(channel);
}

/**
 * @brief   Common CT IRQ handler.
 * @note    It is assumed that the various sources are only activated if the
 *          associated callback pointer is not equal to @p NULL in order to not
 *          perform an extra check in a potentially critical interrupt handler.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_serve_interrupt(PWMDriver *pwmp) {
  uint32_t ris;

  ris  = pwmp->ct->RIS;
  pwmp->ct->IC = ris;
  for (int i=0; i < PWM_CHANNELS; i++) {
    if (((ris & mskCT16_MRnIF(i)) != 0) &&
        (pwmp->config->channels[i].callback != NULL))
      pwmp->config->channels[i].callback(pwmp);
  }
  if (((ris & mskCT16_MRnIF(PWM_CHANNELS)) != 0) && (pwmp->config->callback != NULL))
    pwmp->config->callback(pwmp);
}

#endif /* HAL_USE_PWM */

/** @} */
