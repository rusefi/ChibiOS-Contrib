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
 * @file    USARTv2/hal_serial_lld.c
 * @brief   AT32 low level serial driver code.
 *
 * @addtogroup SERIAL
 * @{
 */

#include "hal.h"

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/* Handling differences in frame size bits.*/
#if !defined(USART_CTRL1_DBN_0)
#define USART_CTRL1_DBN_0                   (1 << 12)
#endif

#if !defined(USART_CTRL1_DBN_1)
#define USART_CTRL1_DBN_1                   (1 << 28)
#endif

/* Workarounds for those devices where UARTs are USARTs.*/
#if defined(USART4)
#define UART4 USART4
#endif
#if defined(USART5)
#define UART5 USART5
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief USART1 serial driver identifier.*/
#if AT32_SERIAL_USE_USART1 || defined(__DOXYGEN__)
SerialDriver SD1;
#endif

/** @brief USART2 serial driver identifier.*/
#if AT32_SERIAL_USE_USART2 || defined(__DOXYGEN__)
SerialDriver SD2;
#endif

/** @brief USART3 serial driver identifier.*/
#if AT32_SERIAL_USE_USART3 || defined(__DOXYGEN__)
SerialDriver SD3;
#endif

/** @brief UART4 serial driver identifier.*/
#if AT32_SERIAL_USE_UART4 || defined(__DOXYGEN__)
SerialDriver SD4;
#endif

/** @brief UART5 serial driver identifier.*/
#if AT32_SERIAL_USE_UART5 || defined(__DOXYGEN__)
SerialDriver SD5;
#endif

/** @brief USART6 serial driver identifier.*/
#if AT32_SERIAL_USE_USART6 || defined(__DOXYGEN__)
SerialDriver SD6;
#endif

/** @brief UART7 serial driver identifier.*/
#if AT32_SERIAL_USE_UART7 || defined(__DOXYGEN__)
SerialDriver SD7;
#endif

/** @brief UART8 serial driver identifier.*/
#if AT32_SERIAL_USE_UART8 || defined(__DOXYGEN__)
SerialDriver SD8;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/** @brief Driver default configuration.*/
static const SerialConfig default_config =
{
  SERIAL_DEFAULT_BITRATE,
  0,
  USART_CTRL2_STOPBN1_BITS,
  0
};

#if AT32_SERIAL_USE_USART1 || defined(__DOXYGEN__)
/** @brief Input buffer for SD1.*/
static uint8_t sd_in_buf1[AT32_SERIAL_USART1_IN_BUF_SIZE];

/** @brief Output buffer for SD1.*/
static uint8_t sd_out_buf1[AT32_SERIAL_USART1_OUT_BUF_SIZE];
#endif

#if AT32_SERIAL_USE_USART2 || defined(__DOXYGEN__)
/** @brief Input buffer for SD2.*/
static uint8_t sd_in_buf2[AT32_SERIAL_USART2_IN_BUF_SIZE];

/** @brief Output buffer for SD2.*/
static uint8_t sd_out_buf2[AT32_SERIAL_USART2_OUT_BUF_SIZE];
#endif

#if AT32_SERIAL_USE_USART3 || defined(__DOXYGEN__)
/** @brief Input buffer for SD3.*/
static uint8_t sd_in_buf3[AT32_SERIAL_USART3_IN_BUF_SIZE];

/** @brief Output buffer for SD3.*/
static uint8_t sd_out_buf3[AT32_SERIAL_USART3_OUT_BUF_SIZE];
#endif

#if AT32_SERIAL_USE_UART4 || defined(__DOXYGEN__)
/** @brief Input buffer for SD4.*/
static uint8_t sd_in_buf4[AT32_SERIAL_UART4_IN_BUF_SIZE];

/** @brief Output buffer for SD4.*/
static uint8_t sd_out_buf4[AT32_SERIAL_UART4_OUT_BUF_SIZE];
#endif

#if AT32_SERIAL_USE_UART5 || defined(__DOXYGEN__)
/** @brief Input buffer for SD5.*/
static uint8_t sd_in_buf5[AT32_SERIAL_UART5_IN_BUF_SIZE];

/** @brief Output buffer for SD5.*/
static uint8_t sd_out_buf5[AT32_SERIAL_UART5_OUT_BUF_SIZE];
#endif

#if AT32_SERIAL_USE_USART6 || defined(__DOXYGEN__)
/** @brief Input buffer for SD6.*/
static uint8_t sd_in_buf6[AT32_SERIAL_USART6_IN_BUF_SIZE];

/** @brief Output buffer for SD6.*/
static uint8_t sd_out_buf6[AT32_SERIAL_USART6_OUT_BUF_SIZE];
#endif

#if AT32_SERIAL_USE_UART7 || defined(__DOXYGEN__)
/** @brief Input buffer for SD7.*/
static uint8_t sd_in_buf7[AT32_SERIAL_UART7_IN_BUF_SIZE];

/** @brief Output buffer for SD7.*/
static uint8_t sd_out_buf7[AT32_SERIAL_UART7_OUT_BUF_SIZE];
#endif

#if AT32_SERIAL_USE_UART8 || defined(__DOXYGEN__)
/** @brief Input buffer for SD8.*/
static uint8_t sd_in_buf8[AT32_SERIAL_UART8_IN_BUF_SIZE];

/** @brief Output buffer for SD8.*/
static uint8_t sd_out_buf8[AT32_SERIAL_UART8_OUT_BUF_SIZE];
#endif

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   USART initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration
 */
static void usart_init(SerialDriver *sdp,
                       const SerialConfig *config) {
  uint32_t baudr, clock;
  USART_TypeDef *u = sdp->usart;

  /* Baud rate setting.*/
  clock = sdp->clock;
    baudr = (uint32_t)((clock + config->speed / 2) / config->speed);

    osalDbgAssert(baudr < 0x10000, "invalid BAUDR value");

  u->BAUDR = baudr;

  /* Note that some bits are enforced.*/
  u->CTRL2 = config->ctrl2 | USART_CTRL2_BFIEN;
  u->CTRL3 = config->ctrl3 | USART_CTRL3_ERRIEN;
  u->CTRL1 = config->ctrl1 | USART_CTRL1_UEN | USART_CTRL1_PERRIEN |
                             USART_CTRL1_RDBFIEN | USART_CTRL1_TEN |
                             USART_CTRL1_REN;
  #if !defined (AT32F435_437)
  u->IFC = 0xFFFFFFFFU;
  #endif

  /* Deciding mask to be applied on the data register on receive, this is
     required in order to mask out the parity bit.*/
  if ((config->ctrl1 & USART_CTRL1_PEN) != 0U) {
    switch (config->ctrl1 & (USART_CTRL1_DBN_1 | USART_CTRL1_DBN_0)) {
    case 0:
      sdp->rxmask = 0x7F;
      break;
    case USART_CTRL1_DBN_1:
      sdp->rxmask = 0x3F;
      break;
    default:
      sdp->rxmask = 0xFF;
    }
  }
  else {
    sdp->rxmask = 0xFF;
  }
}

/**
 * @brief   USART de-initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] u         pointer to an USART I/O block
 */
static void usart_deinit(USART_TypeDef *u) {

  u->CTRL1 = 0;
  u->CTRL2 = 0;
  u->CTRL3 = 0;
}

/**
 * @brief   Error handling routine.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] sts       USART STS register value
 */
static void set_error(SerialDriver *sdp, uint16_t sts) {
  eventflags_t status = 0;

  if (sts & USART_STS_ROERR)
    status |= SD_OVERRUN_ERROR;
  if (sts & USART_STS_PERR)
    status |= SD_PARITY_ERROR;
  if (sts & USART_STS_FERR)
    status |= SD_FRAMING_ERROR;
  if (sts & USART_STS_NERR)
    status |= SD_NOISE_ERROR;
  osalSysLockFromISR();
  chnAddFlagsI(sdp, status);
  osalSysUnlockFromISR();
}

#if AT32_SERIAL_USE_USART1 || defined(__DOXYGEN__)
static void notify1(io_queue_t *qp) {

  (void)qp;
  USART1->CTRL1 |= USART_CTRL1_TDBEIEN | USART_CTRL1_TDCIEN;
}
#endif

#if AT32_SERIAL_USE_USART2 || defined(__DOXYGEN__)
static void notify2(io_queue_t *qp) {

  (void)qp;
  USART2->CTRL1 |= USART_CTRL1_TDBEIEN | USART_CTRL1_TDCIEN;
}
#endif

#if AT32_SERIAL_USE_USART3 || defined(__DOXYGEN__)
static void notify3(io_queue_t *qp) {

  (void)qp;
  USART3->CTRL1 |= USART_CTRL1_TDBEIEN | USART_CTRL1_TDCIEN;
}
#endif

#if AT32_SERIAL_USE_UART4 || defined(__DOXYGEN__)
static void notify4(io_queue_t *qp) {

  (void)qp;
  UART4->CTRL1 |= USART_CTRL1_TDBEIEN | USART_CTRL1_TDCIEN;
}
#endif

#if AT32_SERIAL_USE_UART5 || defined(__DOXYGEN__)
static void notify5(io_queue_t *qp) {

  (void)qp;
  UART5->CTRL1 |= USART_CTRL1_TDBEIEN | USART_CTRL1_TDCIEN;
}
#endif

#if AT32_SERIAL_USE_USART6 || defined(__DOXYGEN__)
static void notify6(io_queue_t *qp) {

  (void)qp;
  USART6->CTRL1 |= USART_CTRL1_TDBEIEN | USART_CTRL1_TDCIEN;
}
#endif

#if AT32_SERIAL_USE_UART7 || defined(__DOXYGEN__)
static void notify7(io_queue_t *qp) {

  (void)qp;
  UART7->CTRL1 |= USART_CTRL1_TDBEIEN | USART_CTRL1_TDCIEN;
}
#endif

#if AT32_SERIAL_USE_UART8 || defined(__DOXYGEN__)
static void notify8(io_queue_t *qp) {

  (void)qp;
  UART8->CTRL1 |= USART_CTRL1_TDBEIEN | USART_CTRL1_TDCIEN;
}
#endif

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if AT32_SERIAL_USE_USART1 || defined(__DOXYGEN__)
#if !defined(AT32_USART1_SUPPRESS_ISR)
#if !defined(AT32_USART1_HANDLER)
#error "AT32_USART1_HANDLER not defined"
#endif
/**
 * @brief   USART1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(AT32_USART1_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  sd_lld_serve_interrupt(&SD1);

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif

#if AT32_SERIAL_USE_USART2 || defined(__DOXYGEN__)
#if !defined(AT32_USART2_SUPPRESS_ISR)
#if !defined(AT32_USART2_HANDLER)
#error "AT32_USART2_HANDLER not defined"
#endif
/**
 * @brief   USART2 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(AT32_USART2_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  sd_lld_serve_interrupt(&SD2);

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif

#if AT32_SERIAL_USE_USART3 || defined(__DOXYGEN__)
#if !defined(AT32_USART3_SUPPRESS_ISR)
#if !defined(AT32_USART3_HANDLER)
#error "AT32_USART3_HANDLER not defined"
#endif
/**
 * @brief   USART3 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(AT32_USART3_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  sd_lld_serve_interrupt(&SD3);

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif

#if AT32_SERIAL_USE_UART4 || defined(__DOXYGEN__)
#if !defined(AT32_UART4_SUPPRESS_ISR)
#if !defined(AT32_UART4_HANDLER)
#error "AT32_UART4_HANDLER not defined"
#endif
/**
 * @brief   UART4 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(AT32_UART4_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  sd_lld_serve_interrupt(&SD4);

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif

#if AT32_SERIAL_USE_UART5 || defined(__DOXYGEN__)
#if !defined(AT32_UART5_SUPPRESS_ISR)
#if !defined(AT32_UART5_HANDLER)
#error "AT32_UART5_HANDLER not defined"
#endif
/**
 * @brief   UART5 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(AT32_UART5_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  sd_lld_serve_interrupt(&SD5);

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif

#if AT32_SERIAL_USE_USART6 || defined(__DOXYGEN__)
#if !defined(AT32_USART6_SUPPRESS_ISR)
#if !defined(AT32_USART6_HANDLER)
#error "AT32_USART6_HANDLER not defined"
#endif
/**
 * @brief   USART6 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(AT32_USART6_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  sd_lld_serve_interrupt(&SD6);

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif

#if AT32_SERIAL_USE_UART7 || defined(__DOXYGEN__)
#if !defined(AT32_UART7_SUPPRESS_ISR)
#if !defined(AT32_UART7_HANDLER)
#error "AT32_UART7_HANDLER not defined"
#endif
/**
 * @brief   UART7 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(AT32_UART7_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  sd_lld_serve_interrupt(&SD7);

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif

#if AT32_SERIAL_USE_UART8 || defined(__DOXYGEN__)
#if !defined(AT32_UART8_SUPPRESS_ISR)
#if !defined(AT32_UART8_HANDLER)
#error "AT32_UART8_HANDLER not defined"
#endif
/**
 * @brief   UART8 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(AT32_UART8_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  sd_lld_serve_interrupt(&SD8);

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level serial driver initialization.
 *
 * @notapi
 */
void sd_lld_init(void) {

#if AT32_SERIAL_USE_USART1
  sdObjectInit(&SD1);
  iqObjectInit(&SD1.iqueue, sd_in_buf1, sizeof sd_in_buf1, NULL, &SD1);
  oqObjectInit(&SD1.oqueue, sd_out_buf1, sizeof sd_out_buf1, notify1, &SD1);
  SD1.usart = USART1;
  SD1.clock = AT32_PCLK2;
#if !defined(AT32_USART1_SUPPRESS_ISR) && defined(AT32_USART1_NUMBER)
  nvicEnableVector(AT32_USART1_NUMBER, AT32_SERIAL_USART1_PRIORITY);
#endif
#endif

#if AT32_SERIAL_USE_USART2
  sdObjectInit(&SD2);
  iqObjectInit(&SD2.iqueue, sd_in_buf2, sizeof sd_in_buf2, NULL, &SD2);
  oqObjectInit(&SD2.oqueue, sd_out_buf2, sizeof sd_out_buf2, notify2, &SD2);
  SD2.usart = USART2;
  SD2.clock = AT32_PCLK1;
#if !defined(AT32_USART2_SUPPRESS_ISR) && defined(AT32_USART2_NUMBER)
  nvicEnableVector(AT32_USART2_NUMBER, AT32_SERIAL_USART2_PRIORITY);
#endif
#endif

#if AT32_SERIAL_USE_USART3
  sdObjectInit(&SD3);
  iqObjectInit(&SD3.iqueue, sd_in_buf3, sizeof sd_in_buf3, NULL, &SD3);
  oqObjectInit(&SD3.oqueue, sd_out_buf3, sizeof sd_out_buf3, notify3, &SD3);
  SD3.usart = USART3;
  SD3.clock = AT32_PCLK1;
#if !defined(AT32_USART3_SUPPRESS_ISR) && defined(AT32_USART3_NUMBER)
  nvicEnableVector(AT32_USART3_NUMBER, AT32_SERIAL_USART3_PRIORITY);
#endif
#endif

#if AT32_SERIAL_USE_UART4
  sdObjectInit(&SD4);
  iqObjectInit(&SD4.iqueue, sd_in_buf4, sizeof sd_in_buf4, NULL, &SD4);
  oqObjectInit(&SD4.oqueue, sd_out_buf4, sizeof sd_out_buf4, notify4, &SD4);
  SD4.usart = UART4;
  SD4.clock = AT32_PCLK1;
#if !defined(AT32_UART4_SUPPRESS_ISR) && defined(AT32_UART4_NUMBER)
  nvicEnableVector(AT32_UART4_NUMBER, AT32_SERIAL_UART4_PRIORITY);
#endif
#endif

#if AT32_SERIAL_USE_UART5
  sdObjectInit(&SD5);
  iqObjectInit(&SD5.iqueue, sd_in_buf5, sizeof sd_in_buf5, NULL, &SD5);
  oqObjectInit(&SD5.oqueue, sd_out_buf5, sizeof sd_out_buf5, notify5, &SD5);
  SD5.usart = UART5;
  SD5.clock = AT32_PCLK1;
#if !defined(AT32_UART5_SUPPRESS_ISR) && defined(AT32_UART5_NUMBER)
  nvicEnableVector(AT32_UART5_NUMBER, AT32_SERIAL_UART5_PRIORITY);
#endif
#endif

#if AT32_SERIAL_USE_USART6
  sdObjectInit(&SD6);
  iqObjectInit(&SD6.iqueue, sd_in_buf6, sizeof sd_in_buf6, NULL, &SD6);
  oqObjectInit(&SD6.oqueue, sd_out_buf6, sizeof sd_out_buf6, notify6, &SD6);
  SD6.usart = USART6;
  SD6.clock = AT32_PCLK2;
#if !defined(AT32_USART6_SUPPRESS_ISR) && defined(AT32_USART6_NUMBER)
  nvicEnableVector(AT32_USART6_NUMBER, AT32_SERIAL_USART6_PRIORITY);
#endif
#endif

#if AT32_SERIAL_USE_UART7
  sdObjectInit(&SD7);
  iqObjectInit(&SD7.iqueue, sd_in_buf7, sizeof sd_in_buf7, NULL, &SD7);
  oqObjectInit(&SD7.oqueue, sd_out_buf7, sizeof sd_out_buf7, notify7, &SD7);
  SD7.usart = UART7;
  SD7.clock = AT32_PCLK1;
#if !defined(AT32_UART7_SUPPRESS_ISR) && defined(AT32_UART7_NUMBER)
  nvicEnableVector(AT32_UART7_NUMBER, AT32_SERIAL_UART7_PRIORITY);
#endif
#endif

#if AT32_SERIAL_USE_UART8
  sdObjectInit(&SD8);
  iqObjectInit(&SD8.iqueue, sd_in_buf8, sizeof sd_in_buf8, NULL, &SD8);
  oqObjectInit(&SD8.oqueue, sd_out_buf8, sizeof sd_out_buf8, notify8, &SD8);
  SD8.usart = UART8;
  SD8.clock = AT32_PCLK1;
#if !defined(AT32_UART8_SUPPRESS_ISR) && defined(AT32_UART8_NUMBER)
  nvicEnableVector(AT32_UART8_NUMBER, AT32_SERIAL_UART8_PRIORITY);
#endif
#endif
}

/**
 * @brief   Low level serial driver configuration and (re)start.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration.
 *                      If this parameter is set to @p NULL then a default
 *                      configuration is used.
 *
 * @notapi
 */
void sd_lld_start(SerialDriver *sdp, const SerialConfig *config) {

  if (config == NULL)
    config = &default_config;

  if (sdp->state == SD_STOP) {
#if AT32_SERIAL_USE_USART1
    if (&SD1 == sdp) {
      crmEnableUSART1(true);
    }
#endif
#if AT32_SERIAL_USE_USART2
    if (&SD2 == sdp) {
      crmEnableUSART2(true);
    }
#endif
#if AT32_SERIAL_USE_USART3
    if (&SD3 == sdp) {
      crmEnableUSART3(true);
    }
#endif
#if AT32_SERIAL_USE_UART4
    if (&SD4 == sdp) {
      crmEnableUART4(true);
    }
#endif
#if AT32_SERIAL_USE_UART5
    if (&SD5 == sdp) {
      crmEnableUART5(true);
    }
#endif
#if AT32_SERIAL_USE_USART6
    if (&SD6 == sdp) {
      crmEnableUSART6(true);
    }
#endif
#if AT32_SERIAL_USE_UART7
    if (&SD7 == sdp) {
      crmEnableUART7(true);
    }
#endif
#if AT32_SERIAL_USE_UART8
    if (&SD8 == sdp) {
      crmEnableUART8(true);
    }
#endif
  }
  usart_init(sdp, config);
}

/**
 * @brief   Low level serial driver stop.
 * @details De-initializes the USART, stops the associated clock, resets the
 *          interrupt vector.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 *
 * @notapi
 */
void sd_lld_stop(SerialDriver *sdp) {

  if (sdp->state == SD_READY) {
    /* UART is de-initialized then clocks are disabled.*/
    usart_deinit(sdp->usart);

#if AT32_SERIAL_USE_USART1
    if (&SD1 == sdp) {
      crmDisableUSART1();
      return;
    }
#endif
#if AT32_SERIAL_USE_USART2
    if (&SD2 == sdp) {
      crmDisableUSART2();
      return;
    }
#endif
#if AT32_SERIAL_USE_USART3
    if (&SD3 == sdp) {
      crmDisableUSART3();
      return;
    }
#endif
#if AT32_SERIAL_USE_UART4
    if (&SD4 == sdp) {
      crmDisableUART4();
      return;
    }
#endif
#if AT32_SERIAL_USE_UART5
    if (&SD5 == sdp) {
      crmDisableUART5();
      return;
    }
#endif
#if AT32_SERIAL_USE_USART6
    if (&SD6 == sdp) {
      crmDisableUSART6();
      return;
    }
#endif
#if AT32_SERIAL_USE_UART7
    if (&SD7 == sdp) {
      crmDisableUART7();
      return;
    }
#endif
#if AT32_SERIAL_USE_UART8
    if (&SD8 == sdp) {
      crmDisableUART8();
      return;
    }
#endif
  }
}

/**
 * @brief   Common IRQ handler.
 *
 * @param[in] sdp       communication channel associated to the USART
 */
void sd_lld_serve_interrupt(SerialDriver *sdp) {
  USART_TypeDef *u = sdp->usart;
  uint32_t ctrl1;
  uint32_t sts;

  /* Reading and clearing status.*/
  sts = u->STS;
#if !defined (AT32F435_437)
  u->IFC = sts;
#endif
  /* Error condition detection.*/
  if (sts & (USART_STS_ROERR | USART_STS_NERR | USART_STS_FERR | USART_STS_PERR))
    set_error(sdp, sts);

  /* Special case, LIN break detection.*/
  if (sts & USART_STS_BFF) {
    osalSysLockFromISR();
    chnAddFlagsI(sdp, SD_BREAK_DETECTED);
    osalSysUnlockFromISR();
  }

  /* Data available, note it is a while in order to handle two situations:
     1) Another byte arrived after removing the previous one, this would cause
        an extra interrupt to serve.
     2) FIFO mode is enabled on devices that support it, we need to empty
        the FIFO.*/
  while (sts & USART_STS_RDBF) {
    osalSysLockFromISR();
    sdIncomingDataI(sdp, (uint8_t)u->DT & sdp->rxmask);
    osalSysUnlockFromISR();

    sts = u->STS;
  }

  /* Caching CTRL1.*/
  ctrl1 = u->CTRL1;

  /* Transmission buffer empty, note it is a while in order to handle two
     situations:
     1) The data registers has been emptied immediately after writing it, this
        would cause an extra interrupt to serve.
     2) FIFO mode is enabled on devices that support it, we need to fill
        the FIFO.*/
  if (ctrl1 & USART_CTRL1_TDBEIEN) {
    while (sts & USART_STS_TDBE) {
      msg_t b;

      osalSysLockFromISR();
      b = oqGetI(&sdp->oqueue);
      if (b < MSG_OK) {
        chnAddFlagsI(sdp, CHN_OUTPUT_EMPTY);
        ctrl1 &= ~USART_CTRL1_TDBEIEN;
        osalSysUnlockFromISR();
        break;
      }
      u->DT = b;
      osalSysUnlockFromISR();

      sts = u->STS;
    }
  }

  /* Physical transmission end.*/
  if ((ctrl1 & USART_CTRL1_TDCIEN) && (sts & USART_STS_TDC)) {
    osalSysLockFromISR();
    if (oqIsEmptyI(&sdp->oqueue)) {
      chnAddFlagsI(sdp, CHN_TRANSMISSION_END);
      ctrl1 &= ~USART_CTRL1_TDCIEN;
    }
    osalSysUnlockFromISR();
  }

  /* Writing CTRL1 once.*/
  u->CTRL1 = ctrl1;
}

#endif /* HAL_USE_SERIAL */

/** @} */
