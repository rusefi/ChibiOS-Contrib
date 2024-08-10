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
 * @file    CT/sn32_ct.h
 * @brief   SN32 CT units common header.
 * @note    This file requires definitions from the SN32 header file.
 *
 * @{
 */

#ifndef SN32_CT_H
#define SN32_CT_H

#include <SN32F2xx.h>

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    CT units references
 * @{
 */
#define SN32_CT16B0       ((sn32_ct_t *)SN_CT16B0_BASE)
#define SN32_CT16B1       ((sn32_ct_t *)SN_CT16B1_BASE)
#define SN32_CT16B1_MAX_CHANNELS       25
#if defined(SN32F240)
#   define SN32_CT16B1_CHANNELS        4
#elif (defined(SN32F240B)|| defined(SN32F240C))
#   define SN32_CT16B1_CHANNELS        25
#elif defined(SN32F260)
#   define SN32_CT16B1_CHANNELS        24
#elif (defined(SN32F280) || defined(SN32F290))
#   define SN32_CT16B1_CHANNELS        12
#else
#   error "CT not supported in the selected device"
#endif
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   SN32 CT registers block.
 * @note    This is the most general known form, not all timers have
 *          necessarily all registers and bits.
 */
typedef struct {                                    /*!< (@ 0x40002000) SN_CT16Bn Structure                                        */
    volatile uint32_t TMRCTRL;                      /*!< (@ 0x00000000) Offset:0x00 CT16Bn Timer Control Register                  */
    volatile uint32_t TC;                           /*!< (@ 0x00000004) Offset:0x04 CT16Bn Timer Counter Register                  */
    volatile uint32_t PRE;                          /*!< (@ 0x00000008) Offset:0x08 CT16Bn Prescale Register                       */
    volatile uint32_t PC;                           /*!< (@ 0x0000000C) Offset:0x0C CT16Bn Prescale Counter Register               */
    volatile uint32_t CNTCTRL;                      /*!< (@ 0x00000010) Offset:0x10 CT16Bn Counter Control Register                */
    volatile uint32_t MCTRL;                        /*!< (@ 0x00000014) Offset:0x14 CT16Bn Match Control Register                  */
    volatile uint32_t MCTRL2;                       /*!< (@ 0x00000018) Offset:0x18 CT16Bn Match Control Register 2                */
    volatile uint32_t MCTRL3;                       /*!< (@ 0x0000001C) Offset:0x1C CT16Bn Match Control Register 3                */
    volatile uint32_t MR[SN32_CT16B1_MAX_CHANNELS]; /*!< (@ 0x00000020) Offset:0x20 CT16Bn MR0 Register                            */
#if (SN32_CT16B1_CHANNELS != SN32_CT16B1_MAX_CHANNELS)
    volatile const uint32_t RESERVED[SN32_CT16B1_MAX_CHANNELS - SN32_CT16B1_CHANNELS];
#endif
    volatile uint32_t CAP0;                         /*!< (@ 0x00000084) Offset:0x84 CT16Bn CAP0 Register                           */
    volatile uint32_t EM;                           /*!< (@ 0x00000088) Offset:0x88 CT16Bn External Match Register                 */
    volatile uint32_t EMC;                          /*!< (@ 0x0000008C) Offset:0x8C CT16Bn External Match Control register         */
    volatile uint32_t EMC2;                         /*!< (@ 0x00000090) Offset:0x90 CT16Bn External Match Control register 2       */
    volatile uint32_t PWMCTRL;                      /*!< (@ 0x00000094) Offset:0x94 CT16Bn PWM Control Register                    */
    volatile uint32_t PWMCTRL2;                     /*!< (@ 0x00000098) Offset:0x98 CT16Bn PWM Control Register 2                  */
    volatile uint32_t PWMENB;                       /*!< (@ 0x0000009C) Offset:0x9C CT16Bn PWM Enable register                     */
    volatile uint32_t PWMIOENB;                     /*!< (@ 0x000000A0) Offset:0xA0 CT16Bn PWM IO Enable register                  */
    volatile uint32_t RIS;                          /*!< (@ 0x000000A4) Offset:0xA4 CT16Bn Raw Interrupt Status Register           */
    volatile uint32_t IC;                           /*!< (@ 0x000000A8) Offset:0xA8 CT16Bn Interrupt Clear Register                */
} sn32_ct_t;                                        /*!< Size = 172 (0xac)                                                         */

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/
/* CT16Bn Timer Control register <CT16Bn_TMRCTRL> (0x00) */
#define CT16_CEN_DIS                   0 //[0:0] CT16Bn enable bit
#define CT16_CEN_EN                    1
#define mskCT16_CEN_DIS                (CT16_CEN_DIS<<0)
#define mskCT16_CEN_EN                 (CT16_CEN_EN<<0)

#define CT16_CRST                      1 //[1:1] CT16Bn counter reset bit
#define mskCT16_CRST                   (CT16_CRST<<1)

//[6:4] CT16Bn counting mode selection
#define CT16_CM_EDGE_UP                0 // Edge-aligned Up-counting mode
#define CT16_CM_EDGE_DOWN              1 // Edge-aligned Down-counting mode
#define CT16_CM_CENTER_UP              2 // Center-aligned mode 1. Match interrupt is set during up-counting period
#define CT16_CM_CENTER_DOWN            4 // Center-aligned mode 2. Match interrupt is set during down-counting period
#define CT16_CM_CENTER_BOTH            6 // Center-aligned mode 3. Match interrupt is set during both up and down period.
#define mskCT16_CM_EDGE_UP             (CT16_CM_EDGE_UP<<4)
#define mskCT16_CM_EDGE_DOWN           (CT16_CM_EDGE_DOWN<<4)
#define mskCT16_CM_CENTER_UP           (CT16_CM_CENTER_UP<<4)
#define mskCT16_CM_CENTER_DOWN         (CT16_CM_CENTER_DOWN<<4)
#define mskCT16_CM_CENTER_BOTH         (CT16_CM_CENTER_BOTH<<4)

/* CT16Bn Count Control register <CT16Bn_CNTCTRL> (0x10) */
//[1:0] Count/Timer Mode selection.
#define CT16_CTM_TIMER                 0 // Timer mode: Every rising PCLK edge.
#define CT16_CTM_CNTER_RISING          1 // Counter mode: TC increments on rising edge of CAP input.
#define CT16_CTM_CNTER_FALLING         2 // Counter mode: TC increments on falling edge of CAP input.
#define CT16_CTM_CNTER_BOTH            3 // Counter mode: TC increments on both edge of CAP input.
#define mskCT16_CTM_TIMER              (CT16_CTM_TIMER<<0)
#define mskCT16_CTM_CNTER_RISING       (CT16_CTM_CNTER_RISING<<0)
#define mskCT16_CTM_CNTER_FALLING      (CT16_CTM_CNTER_FALLING<<0)
#define mskCT16_CTM_CNTER_BOTH         (CT16_CTM_CNTER_BOTH<<0)

#define CT16_CIS                       0 //[3:2] Count Input Select
#define mskCT16_CIS                    (CT16_CIS<<2)

/* CT16Bn Match Control register <CT16Bn_MCTRL> (0x14) */
#define CT16_MRnIE_EN                  1 // Enable MRn match interrupt
#define CT16_MRnIE_DIS                 0
#define mskCT16_MRnIE_EN(n)            (CT16_MRnIE_EN<<((n % 10) *3))
#define mskCT16_MRnIE_DIS(n)           (CT16_MRnIE_DIS<<((n % 10) *3))

#define CT16_MRnRST_EN                 1 // Enable reset TC when MRn matches TC.
#define CT16_MRnRST_DIS                0
#define mskCT16_MRnRST_EN(n)           (CT16_MRnRST_EN<<(((n % 10) *3) +1))
#define mskCT16_MRnRST_DIS(n)          (CT16_MRnRST_DIS<<(((n % 10) *3) +1))

#define CT16_MRnSTOP_EN                1 // Enable stop TC and clear CEN when MRn matches TC.
#define CT16_MRnSTOP_DIS               0
#define mskCT16_MRnSTOP_EN(n)          (CT16_MRnSTOP_EN<<(((n % 10) *3) +2))
#define mskCT16_MRnSTOP_DIS(n)         (CT16_MRnSTOP_DIS<<(((n % 10) *3) +2))

/* CT16Bn Capture Control register <CT16Bn_CAPCTRL> (0x80) */
#define CT16_CAP0RE_EN                 1 //[0:0] Enable CAP0 capture on rising edge.
#define CT16_CAP0RE_DIS                0
#define mskCT16_CAP0RE_EN              (CT16_CAP0RE_EN<<0)
#define mskCT16_CAP0RE_DIS             (CT16_CAP0RE_DIS<<0)

#define CT16_CAP0FE_EN                 1 //[1:1] Enable CAP0 capture on fallng edge.
#define CT16_CAP0FE_DIS                0
#define mskCT16_CAP0FE_EN              (CT16_CAP0FE_EN<<1)
#define mskCT16_CAP0FE_DIS             (CT16_CAP0FE_DIS<<1)

#define CT16_CAP0IE_EN                 1 //[2:2] Enable CAP0 interrupt.
#define CT16_CAP0IE_DIS                0
#define mskCT16_CAP0IE_EN              (CT16_CAP0IE_EN<<2)
#define mskCT16_CAP0IE_DIS             (CT16_CAP0IE_DIS<<2)

#define CT16_CAP0EN_EN                 1 //[3:3] Enable CAP0 function.
#define CT16_CAP0EN_DIS                0
#define mskCT16_CAP0EN_EN              (CT16_CAP0EN_EN<<3)
#define mskCT16_CAP0EN_DIS             (CT16_CAP0EN_DIS<<3)

/* CT16Bn External Match register <CT16Bn_EM> (0x88) */
#define CT16_EMn                       1 // CT16Bn PWMn drive state
#define mskCT16_EM(n)                  (CT16_EMn<<n)
/* CT16Bn PWM Control register <CT16Bn_PWMCTRL> (0x94) */
#define CT16_PWMnMODE_1                0 // PWM mode 1.
#define CT16_PWMnMODE_2                1 // PWM mode 2.
#define CT16_PWMnMODE_FORCE_0          2 // Force 0.
#define CT16_PWMnMODE_FORCE_1          3 // Force 1.
#define mskCT16_PWMnMODE_1(n)          (CT16_PWMnMODE_1<<((n % 16) *2))
#define mskCT16_PWMnMODE_2(n)          (CT16_PWMnMODE_2<<((n % 16) *2))
#define mskCT16_PWMnMODE_FORCE_0(n)    (CT16_PWMnMODE_FORCE_0<<((n % 16) *2))
#define mskCT16_PWMnMODE_FORCE_1(n)    (CT16_PWMnMODE_FORCE_1<<((n % 16) *2))
/* CT16Bn PWM Enable register <CT16Bn_PWMENB> (0x9C) */
#define CT16_PWMnEN_EN                 1 // CT16Bn PWMn is enabled for PWM mode.
#define CT16_PWMnEN_EMn                0 // CT16Bn PWMn is controlled by EMn.
#define mskCT16_PWMnEN_EN(n)           (CT16_PWMnEN_EN<<n)
#define mskCT16_PWMnEN_EM0(n)          (CT16_PWMnEN_EM0<<n)
/* CT16Bn PWM IO Enable register <CT16Bn_PWMIOENB> (0xA0) */
#define CT16_PWMnIOEN_EN               1 // PWMn pin acts as match output.
#define CT16_PWMnIOEN_DIS              0 // PWMn pin acts as GPIO.
#define mskCT16_PWMnIOEN_EN(n)         (CT16_PWMnIOEN_EN<<n)
#define mskCT16_PWMnIOEN_DIS(n)        (CT16_PWMnIOEN_DIS<<n)
/* CT16Bn External Match Control register <CT16Bn_EMC> (0x8C) */
#define CT16_EMCn_DO_NOTHING           0   //Do nothing.
#define CT16_EMCn_LOW                  1   //CT16Bn PWM0 pin is low.
#define CT16_EMCn_HIGH                 2   //CT16Bn PWM0 pin is high.
#define CT16_EMCn_TOGGLE               3   //Toggle CT16Bn PWM0 pin.
#define mskCT16_EMCn_DO_NOTHING(n)     (CT16_EMCn_LOW<<((n % 16) *2))
#define mskCT16_EMCn_LOW(n)            (CT16_EMCn_LOW<<((n % 16) *2))
#define mskCT16_EMCn_HIGH(n)           (CT16_EMCn_HIGH<<((n % 16) *2))
#define mskCT16_EMCn_TOGGLE(n)         (CT16_EMCn_TOGGLE<<((n % 16) *2))
/* CT16Bn Timer Raw Interrupt Status register <CT16Bn_RIS> (0xA4) */
/* CT16Bn Timer Interrupt Clear register <CT16Bn_IC> (0xA8) */
/* The bitmask usage of iwCT16Bn_IrqEvent is the same with CT16Bn_RIS*/
#define mskCT16_MRnIF(n)               (0x1<<n) // Interrupt flag for match channel n
#define mskCT16_MRnIC(n)               mskCT16_MRnIF(n)

#define CT16B0_ResetTimer()                                  \
  do {                                                       \
      SN_CT16B0->TMRCTRL = (mskCT16_CRST);                   \
      while (SN_CT16B0->TMRCTRL & mskCT16_CRST);             \
  } while (0)
#define CT16B1_ResetTimer()                                  \
  do {                                                       \
      SN_CT16B1->TMRCTRL = (mskCT16_CRST);                   \
      while (SN_CT16B1->TMRCTRL & mskCT16_CRST);             \
  } while (0)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#endif /* SN32_CT_H */

/** @} */
