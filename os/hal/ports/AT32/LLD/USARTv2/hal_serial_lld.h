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
 * @file    USARTv2/hal_serial_lld.h
 * @brief   AT32 low level serial driver header.
 *
 * @addtogroup SERIAL
 * @{
 */

#ifndef HAL_SERIAL_LLD_H
#define HAL_SERIAL_LLD_H

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

#include "at32_usart.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Advanced buffering support switch.
 * @details This constants enables the advanced buffering support in the
 *          low level driver, the queue buffer is no more part of the
 *          @p SerialDriver structure, each driver can have a different
 *          queue size.
 */
#define SERIAL_ADVANCED_BUFFERING_SUPPORT   TRUE

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   USART1 driver enable switch.
 * @details If set to @p TRUE the support for USART1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(AT32_SERIAL_USE_USART1) || defined(__DOXYGEN__)
#define AT32_SERIAL_USE_USART1              FALSE
#endif

/**
 * @brief   USART2 driver enable switch.
 * @details If set to @p TRUE the support for USART2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(AT32_SERIAL_USE_USART2) || defined(__DOXYGEN__)
#define AT32_SERIAL_USE_USART2              FALSE
#endif

/**
 * @brief   USART3 driver enable switch.
 * @details If set to @p TRUE the support for USART3 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(AT32_SERIAL_USE_USART3) || defined(__DOXYGEN__)
#define AT32_SERIAL_USE_USART3              FALSE
#endif

/**
 * @brief   UART4 driver enable switch.
 * @details If set to @p TRUE the support for UART4 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(AT32_SERIAL_USE_UART4) || defined(__DOXYGEN__)
#define AT32_SERIAL_USE_UART4               FALSE
#endif

/**
 * @brief   UART5 driver enable switch.
 * @details If set to @p TRUE the support for UART5 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(AT32_SERIAL_USE_UART5) || defined(__DOXYGEN__)
#define AT32_SERIAL_USE_UART5               FALSE
#endif

/**
 * @brief   USART6 driver enable switch.
 * @details If set to @p TRUE the support for USART6 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(AT32_SERIAL_USE_USART6) || defined(__DOXYGEN__)
#define AT32_SERIAL_USE_USART6              FALSE
#endif

/**
 * @brief   UART7 driver enable switch.
 * @details If set to @p TRUE the support for UART7 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(AT32_SERIAL_USE_UART7) || defined(__DOXYGEN__)
#define AT32_SERIAL_USE_UART7               FALSE
#endif

/**
 * @brief   UART8 driver enable switch.
 * @details If set to @p TRUE the support for UART8 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(AT32_SERIAL_USE_UART8) || defined(__DOXYGEN__)
#define AT32_SERIAL_USE_UART8               FALSE
#endif

/**
 * @brief   USART1 interrupt priority level setting.
 */
#if !defined(AT32_SERIAL_USART1_PRIORITY) || defined(__DOXYGEN__)
#define AT32_SERIAL_USART1_PRIORITY         12
#endif

/**
 * @brief   USART2 interrupt priority level setting.
 */
#if !defined(AT32_SERIAL_USART2_PRIORITY) || defined(__DOXYGEN__)
#define AT32_SERIAL_USART2_PRIORITY         12
#endif

/**
 * @brief   USART3 interrupt priority level setting.
 */
#if !defined(AT32_SERIAL_USART3_PRIORITY) || defined(__DOXYGEN__)
#define AT32_SERIAL_USART3_PRIORITY         12
#endif

/**
 * @brief   UART4 interrupt priority level setting.
 */
#if !defined(AT32_SERIAL_UART4_PRIORITY) || defined(__DOXYGEN__)
#define AT32_SERIAL_UART4_PRIORITY          12
#endif

/**
 * @brief   UART5 interrupt priority level setting.
 */
#if !defined(AT32_SERIAL_UART5_PRIORITY) || defined(__DOXYGEN__)
#define AT32_SERIAL_UART5_PRIORITY          12
#endif

/**
 * @brief   USART6 interrupt priority level setting.
 */
#if !defined(AT32_SERIAL_USART6_PRIORITY) || defined(__DOXYGEN__)
#define AT32_SERIAL_USART6_PRIORITY         12
#endif

/**
 * @brief   UART7 interrupt priority level setting.
 */
#if !defined(AT32_SERIAL_UART7_PRIORITY) || defined(__DOXYGEN__)
#define AT32_SERIAL_UART7_PRIORITY          12
#endif

/**
 * @brief   UART8 interrupt priority level setting.
 */
#if !defined(AT32_SERIAL_UART8_PRIORITY) || defined(__DOXYGEN__)
#define AT32_SERIAL_UART8_PRIORITY          12
#endif

/**
 * @brief   Input buffer size for USART1.
 */
#if !defined(AT32_SERIAL_USART1_IN_BUF_SIZE) || defined(__DOXYGEN__)
#define AT32_SERIAL_USART1_IN_BUF_SIZE      SERIAL_BUFFERS_SIZE
#endif

/**
 * @brief   Output buffer size for USART1.
 */
#if !defined(AT32_SERIAL_USART1_OUT_BUF_SIZE) || defined(__DOXYGEN__)
#define AT32_SERIAL_USART1_OUT_BUF_SIZE     SERIAL_BUFFERS_SIZE
#endif

/**
 * @brief   Input buffer size for USART2.
 */
#if !defined(AT32_SERIAL_USART2_IN_BUF_SIZE) || defined(__DOXYGEN__)
#define AT32_SERIAL_USART2_IN_BUF_SIZE      SERIAL_BUFFERS_SIZE
#endif

/**
 * @brief   Output buffer size for USART2.
 */
#if !defined(AT32_SERIAL_USART2_OUT_BUF_SIZE) || defined(__DOXYGEN__)
#define AT32_SERIAL_USART2_OUT_BUF_SIZE     SERIAL_BUFFERS_SIZE
#endif

/**
 * @brief   Input buffer size for USART3.
 */
#if !defined(AT32_SERIAL_USART3_IN_BUF_SIZE) || defined(__DOXYGEN__)
#define AT32_SERIAL_USART3_IN_BUF_SIZE      SERIAL_BUFFERS_SIZE
#endif

/**
 * @brief   Output buffer size for USART3.
 */
#if !defined(AT32_SERIAL_USART3_OUT_BUF_SIZE) || defined(__DOXYGEN__)
#define AT32_SERIAL_USART3_OUT_BUF_SIZE     SERIAL_BUFFERS_SIZE
#endif

/**
 * @brief   Input buffer size for UART4.
 */
#if !defined(AT32_SERIAL_UART4_IN_BUF_SIZE) || defined(__DOXYGEN__)
#define AT32_SERIAL_UART4_IN_BUF_SIZE       SERIAL_BUFFERS_SIZE
#endif

/**
 * @brief   Output buffer size for UART4.
 */
#if !defined(AT32_SERIAL_UART4_OUT_BUF_SIZE) || defined(__DOXYGEN__)
#define AT32_SERIAL_UART4_OUT_BUF_SIZE      SERIAL_BUFFERS_SIZE
#endif

/**
 * @brief   Input buffer size for UART5.
 */
#if !defined(AT32_SERIAL_UART5_IN_BUF_SIZE) || defined(__DOXYGEN__)
#define AT32_SERIAL_UART5_IN_BUF_SIZE       SERIAL_BUFFERS_SIZE
#endif

/**
 * @brief   Output buffer size for UART5.
 */
#if !defined(AT32_SERIAL_UART5_OUT_BUF_SIZE) || defined(__DOXYGEN__)
#define AT32_SERIAL_UART5_OUT_BUF_SIZE      SERIAL_BUFFERS_SIZE
#endif

/**
 * @brief   Input buffer size for USART6.
 */
#if !defined(AT32_SERIAL_USART6_IN_BUF_SIZE) || defined(__DOXYGEN__)
#define AT32_SERIAL_USART6_IN_BUF_SIZE      SERIAL_BUFFERS_SIZE
#endif

/**
 * @brief   Output buffer size for USART6.
 */
#if !defined(AT32_SERIAL_USART6_OUT_BUF_SIZE) || defined(__DOXYGEN__)
#define AT32_SERIAL_USART6_OUT_BUF_SIZE     SERIAL_BUFFERS_SIZE
#endif

/**
 * @brief   Input buffer size for UART7.
 */
#if !defined(AT32_SERIAL_UART7_IN_BUF_SIZE) || defined(__DOXYGEN__)
#define AT32_SERIAL_UART7_IN_BUF_SIZE       SERIAL_BUFFERS_SIZE
#endif

/**
 * @brief   Output buffer size for UART7.
 */
#if !defined(AT32_SERIAL_UART7_OUT_BUF_SIZE) || defined(__DOXYGEN__)
#define AT32_SERIAL_UART7_OUT_BUF_SIZE      SERIAL_BUFFERS_SIZE
#endif

/**
 * @brief   Input buffer size for UART8.
 */
#if !defined(AT32_SERIAL_UART8_IN_BUF_SIZE) || defined(__DOXYGEN__)
#define AT32_SERIAL_UART8_IN_BUF_SIZE       SERIAL_BUFFERS_SIZE
#endif

/**
 * @brief   Output buffer size for UART8.
 */
#if !defined(AT32_SERIAL_UART8_OUT_BUF_SIZE) || defined(__DOXYGEN__)
#define AT32_SERIAL_UART8_OUT_BUF_SIZE      SERIAL_BUFFERS_SIZE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if AT32_SERIAL_USE_USART1 && !AT32_HAS_USART1
#error "USART1 not present in the selected device"
#endif

#if AT32_SERIAL_USE_USART2 && !AT32_HAS_USART2
#error "USART2 not present in the selected device"
#endif

#if AT32_SERIAL_USE_USART3 && !AT32_HAS_USART3
#error "USART3 not present in the selected device"
#endif

#if AT32_SERIAL_USE_UART4 && !AT32_HAS_UART4
#error "UART4 not present in the selected device"
#endif

#if AT32_SERIAL_USE_UART5 && !AT32_HAS_UART5
#error "UART5 not present in the selected device"
#endif

#if AT32_SERIAL_USE_USART6 && !AT32_HAS_USART6
#error "USART6 not present in the selected device"
#endif

#if AT32_SERIAL_USE_UART7 && !AT32_HAS_UART7
#error "UART7 not present in the selected device"
#endif

#if AT32_SERIAL_USE_UART8 && !AT32_HAS_UART8
#error "UART8 not present in the selected device"
#endif

#if !AT32_SERIAL_USE_USART1 && !AT32_SERIAL_USE_USART2 &&                   \
    !AT32_SERIAL_USE_USART3 && !AT32_SERIAL_USE_UART4  &&                   \
    !AT32_SERIAL_USE_UART5  && !AT32_SERIAL_USE_USART6 &&                   \
    !AT32_SERIAL_USE_UART7  && !AT32_SERIAL_USE_UART8
#error "SERIAL driver activated but no USART/UART peripheral assigned"
#endif

#if !defined(AT32_USART1_SUPPRESS_ISR) &&                                   \
    AT32_SERIAL_USE_USART1 &&                                               \
    !OSAL_IRQ_IS_VALID_PRIORITY(AT32_SERIAL_USART1_PRIORITY)
#error "Invalid IRQ priority assigned to USART1"
#endif

#if !defined(AT32_USART2_SUPPRESS_ISR) &&                                   \
    AT32_SERIAL_USE_USART2 &&                                               \
    !OSAL_IRQ_IS_VALID_PRIORITY(AT32_SERIAL_USART2_PRIORITY)
#error "Invalid IRQ priority assigned to USART2"
#endif

#if !defined(AT32_USART3_SUPPRESS_ISR) &&                                   \
    AT32_SERIAL_USE_USART3 &&                                               \
    !OSAL_IRQ_IS_VALID_PRIORITY(AT32_SERIAL_USART3_PRIORITY)
#error "Invalid IRQ priority assigned to USART3"
#endif

#if !defined(AT32_UART4_SUPPRESS_ISR) &&                                    \
    AT32_SERIAL_USE_UART4 &&                                                \
    !OSAL_IRQ_IS_VALID_PRIORITY(AT32_SERIAL_UART4_PRIORITY)
#error "Invalid IRQ priority assigned to UART4"
#endif

#if !defined(AT32_UART5_SUPPRESS_ISR) &&                                    \
    AT32_SERIAL_USE_UART5 &&                                                \
    !OSAL_IRQ_IS_VALID_PRIORITY(AT32_SERIAL_UART5_PRIORITY)
#error "Invalid IRQ priority assigned to UART5"
#endif

#if !defined(AT32_USART6_SUPPRESS_ISR) &&                                   \
    AT32_SERIAL_USE_USART6 &&                                               \
    !OSAL_IRQ_IS_VALID_PRIORITY(AT32_SERIAL_USART6_PRIORITY)
#error "Invalid IRQ priority assigned to USART6"
#endif

#if !defined(AT32_UART7_SUPPRESS_ISR) &&                                    \
    AT32_SERIAL_USE_UART7 &&                                                \
    !OSAL_IRQ_IS_VALID_PRIORITY(AT32_SERIAL_UART7_PRIORITY)
#error "Invalid IRQ priority assigned to UART7"
#endif

#if !defined(AT32_UART8_SUPPRESS_ISR) &&                                    \
    AT32_SERIAL_USE_UART8 &&                                                \
    !OSAL_IRQ_IS_VALID_PRIORITY(AT32_SERIAL_UART8_PRIORITY)
#error "Invalid IRQ priority assigned to UART8"
#endif

/* Checks on allocation of USARTx units.*/
#if AT32_SERIAL_USE_USART1
#if defined(AT32_USART1_IS_USED)
#error "SD1 requires USART1 but it is already used"
#else
#define AT32_USART1_IS_USED
#endif
#endif

#if AT32_SERIAL_USE_USART2
#if defined(AT32_USART2_IS_USED)
#error "SD2 requires USART2 but it is already used"
#else
#define AT32_USART2_IS_USED
#endif
#endif

#if AT32_SERIAL_USE_USART3
#if defined(AT32_USART3_IS_USED)
#error "SD3 requires USART3 but it is already used"
#else
#define AT32_USART3_IS_USED
#endif
#endif

#if AT32_SERIAL_USE_UART4
#if defined(AT32_UART4_IS_USED)
#error "SD4 requires UART4 but it is already used"
#else
#define AT32_UART4_IS_USED
#endif
#endif

#if AT32_SERIAL_USE_UART5
#if defined(AT32_UART5_IS_USED)
#error "SD5 requires UART5 but it is already used"
#else
#define AT32_UART5_IS_USED
#endif
#endif

#if AT32_SERIAL_USE_USART6
#if defined(AT32_USART6_IS_USED)
#error "SD6 requires USART6 but it is already used"
#else
#define AT32_USART6_IS_USED
#endif
#endif

#if AT32_SERIAL_USE_UART7
#if defined(AT32_UART7_IS_USED)
#error "SD7 requires UART7 but it is already used"
#else
#define AT32_UART7_IS_USED
#endif
#endif

#if AT32_SERIAL_USE_UART8
#if defined(AT32_UART8_IS_USED)
#error "SD8 requires UART8 but it is already used"
#else
#define AT32_UART8_IS_USED
#endif
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   AT32 Serial Driver configuration structure.
 * @details An instance of this structure must be passed to @p sdStart()
 *          in order to configure and start a serial driver operations.
 * @note    This structure content is architecture dependent, each driver
 *          implementation defines its own version and the custom static
 *          initializers.
 */
typedef struct hal_serial_config {
  /**
   * @brief Bit rate.
   */
  uint32_t                  speed;
  /* End of the mandatory fields.*/
  /**
   * @brief Initialization value for the CTRL1 register.
   */
  uint32_t                  ctrl1;
  /**
   * @brief Initialization value for the CTRL2 register.
   */
  uint32_t                  ctrl2;
  /**
   * @brief Initialization value for the CTRL3 register.
   */
  uint32_t                  ctrl3;
} SerialConfig;

/**
 * @brief   @p SerialDriver specific data.
 */
#define _serial_driver_data                                                 \
  _base_asynchronous_channel_data                                           \
  /* Driver state.*/                                                        \
  sdstate_t                 state;                                          \
  /* Input queue.*/                                                         \
  input_queue_t             iqueue;                                         \
  /* Output queue.*/                                                        \
  output_queue_t            oqueue;                                         \
  /* End of the mandatory fields.*/                                         \
  /* Pointer to the USART registers block.*/                                \
  USART_TypeDef             *usart;                                         \
  /* Clock frequency for the associated USART/UART.*/                       \
  uint32_t                  clock;                                          \
  /* Mask to be applied on received frames.*/                               \
  uint8_t                   rxmask;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if AT32_SERIAL_USE_USART1 && !defined(__DOXYGEN__)
extern SerialDriver SD1;
#endif
#if AT32_SERIAL_USE_USART2 && !defined(__DOXYGEN__)
extern SerialDriver SD2;
#endif
#if AT32_SERIAL_USE_USART3 && !defined(__DOXYGEN__)
extern SerialDriver SD3;
#endif
#if AT32_SERIAL_USE_UART4 && !defined(__DOXYGEN__)
extern SerialDriver SD4;
#endif
#if AT32_SERIAL_USE_UART5 && !defined(__DOXYGEN__)
extern SerialDriver SD5;
#endif
#if AT32_SERIAL_USE_USART6 && !defined(__DOXYGEN__)
extern SerialDriver SD6;
#endif
#if AT32_SERIAL_USE_UART7 && !defined(__DOXYGEN__)
extern SerialDriver SD7;
#endif
#if AT32_SERIAL_USE_UART8 && !defined(__DOXYGEN__)
extern SerialDriver SD8;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void sd_lld_init(void);
  void sd_lld_start(SerialDriver *sdp, const SerialConfig *config);
  void sd_lld_stop(SerialDriver *sdp);
  void sd_lld_serve_interrupt(SerialDriver *sdp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SERIAL */

#endif /* HAL_SERIAL_LLD_H */

/** @} */
