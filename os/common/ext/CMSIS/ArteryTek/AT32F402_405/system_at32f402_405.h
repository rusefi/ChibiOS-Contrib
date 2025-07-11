/**
  **************************************************************************
  * @file    system_at32f402_405.h
  * @author  Artery Technology & HorrorTroll & Zhaqian & Maxjta
  * @brief   CMSIS AT32F402_405 system header file
  *
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup AT32F402_405_system
  * @{
  */

/**
  * @brief Define to prevent recursive inclusion
  */

#ifndef __SYSTEM_AT32F402_405_H
#define __SYSTEM_AT32F402_405_H

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup AT32F402_405_System_Includes
  * @{
  */

/**
  * @}
  */

/** @defgroup AT32F402_405_System_Exported_Variables
  * @{
  */

extern uint32_t SystemCoreClock;          /*!< System Clock Frequency (Core Clock) */

/**
  * @}
  */

/** @addtogroup AT32F402_405_System_Exported_Constants
  * @{
  */

/**
  * @}
  */

/** @addtogroup AT32F402_405_System_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup AT32F402_405_System_Exported_Functions
  * @{
  */

extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /*__SYSTEM_AT32F402_405_H */

/**
  * @}
  */

/**
  * @}
  */

/*********************** (C) COPYRIGHT Artery Technology *****END OF FILE****/
