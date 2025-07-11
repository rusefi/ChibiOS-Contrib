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
 * @file    USARTv2/hal_uart_lld.c
 * @brief   AT32 low level UART driver code.
 *
 * @addtogroup UART
 * @{
 */

#include "hal.h"

#if HAL_USE_UART || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

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

/** @brief USART1 UART driver identifier.*/
#if AT32_UART_USE_USART1 || defined(__DOXYGEN__)
UARTDriver UARTD1;
#endif

/** @brief USART2 UART driver identifier.*/
#if AT32_UART_USE_USART2 || defined(__DOXYGEN__)
UARTDriver UARTD2;
#endif

/** @brief USART3 UART driver identifier.*/
#if AT32_UART_USE_USART3 || defined(__DOXYGEN__)
UARTDriver UARTD3;
#endif

/** @brief UART4 UART driver identifier.*/
#if AT32_UART_USE_UART4 || defined(__DOXYGEN__)
UARTDriver UARTD4;
#endif

/** @brief UART5 UART driver identifier.*/
#if AT32_UART_USE_UART5 || defined(__DOXYGEN__)
UARTDriver UARTD5;
#endif

/** @brief USART6 UART driver identifier.*/
#if AT32_UART_USE_USART6 || defined(__DOXYGEN__)
UARTDriver UARTD6;
#endif

/** @brief UART7 UART driver identifier.*/
#if AT32_UART_USE_UART7 || defined(__DOXYGEN__)
UARTDriver UARTD7;
#endif

/** @brief UART8 UART driver identifier.*/
#if AT32_UART_USE_UART8 || defined(__DOXYGEN__)
UARTDriver UARTD8;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Status bits translation.
 *
 * @param[in] sts       USART STS register value
 *
 * @return  The error flags.
 */
static uartflags_t translate_errors(uint32_t sts) {
  uartflags_t status = 0;

  if (sts & USART_STS_ROERR)
    status |= UART_OVERRUN_ERROR;
  if (sts & USART_STS_PERR)
    status |= UART_PARITY_ERROR;
  if (sts & USART_STS_FERR)
    status |= UART_FRAMING_ERROR;
  if (sts & USART_STS_NERR)
    status |= UART_NOISE_ERROR;
  if (sts & USART_STS_BFF)
    status |= UART_BREAK_DETECTED;
  return status;
}

/**
 * @brief   Puts the receiver in the UART_RX_IDLE state.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void uart_enter_rx_idle_loop(UARTDriver *uartp) {
  uint32_t mode;

  /* RX DMA channel preparation, if the char callback is defined then the
     FDTIEN interrupt is enabled too.*/
  if (uartp->config->rxchar_cb == NULL)
    mode = AT32_DMA_CCTRL_DTD_P2M | AT32_DMA_CCTRL_LM;
  else
    mode = AT32_DMA_CCTRL_DTD_P2M | AT32_DMA_CCTRL_LM | AT32_DMA_CCTRL_FDTIEN;
  dmaStreamSetMemory0(uartp->dmarx, &uartp->rxbuf);
  dmaStreamSetTransactionSize(uartp->dmarx, 1);
  dmaStreamSetMode(uartp->dmarx, uartp->dmarxmode | mode);
  dmaStreamEnable(uartp->dmarx);
}

/**
 * @brief   USART de-initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void usart_stop(UARTDriver *uartp) {

  /* Stops RX and TX DMA channels.*/
  dmaStreamDisable(uartp->dmarx);
  dmaStreamDisable(uartp->dmatx);

  /* Stops USART operations.*/
  uartp->usart->CTRL1 = 0;
  uartp->usart->CTRL2 = 0;
  uartp->usart->CTRL3 = 0;
}

/**
 * @brief   USART initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void usart_start(UARTDriver *uartp) {
  uint32_t baudr;
  uint32_t ctrl1;
#if !defined (AT32F435_437)
  const uint32_t tmo = uartp->config->timeout;
#endif
  USART_TypeDef *u = uartp->usart;

  /* Defensive programming, starting from a clean state.*/
  usart_stop(uartp);

  /* Baud rate setting.*/
  baudr = (uint32_t)((uartp->clock + uartp->config->speed / 2) /
                     uartp->config->speed);
  u->BAUDR = baudr;

 #if !defined (AT32F435_437)
  /* Resetting eventual pending status flags.*/
  u->IFC = 0xFFFFFFFFU;
 #endif
  /* Note that some bits are enforced because required for correct driver
     operations.*/
  u->CTRL2 = uartp->config->ctrl2 | USART_CTRL2_BFIEN;
  u->CTRL3 = uartp->config->ctrl3 | USART_CTRL3_DMATEN | USART_CTRL3_DMAREN |
                                    USART_CTRL3_ERRIEN;

  /* Mustn't ever set FDTIEN here - if done, it causes an immediate
     interrupt.*/
  ctrl1 = USART_CTRL1_UEN | USART_CTRL1_PERRIEN | USART_CTRL1_TEN | USART_CTRL1_REN;
  u->CTRL1 = uartp->config->ctrl1 | ctrl1;

  /* Set receive timeout and checks if it is really applied.*/
#if !defined (AT32F435_437)
  if (tmo > 0) {
    osalDbgAssert(tmo <= USART_RTOV_RTOV, "Timeout overflow");
    u->RTOV = tmo;
    osalDbgAssert(tmo == u->RTOV, "Timeout feature unsupported in this UART");
  }
#endif
  /* Starting the receiver idle loop.*/
  uart_enter_rx_idle_loop(uartp);
}

/**
 * @brief   RX DMA common service routine.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in] flags     pre-shifted content of the STS register
 */
static void uart_lld_serve_rx_end_irq(UARTDriver *uartp, uint32_t flags) {

  /* DMA errors handling.*/
#if defined(AT32_UART_DMA_ERROR_HOOK)
  if ((flags & AT32_DMA_STS_DTERRF) != 0) {
    AT32_UART_DMA_ERROR_HOOK(uartp);
  }
#else
  (void)flags;
#endif

  if (uartp->rxstate == UART_RX_IDLE) {
    /* Receiver in idle state, a callback is generated, if enabled, for each
       received character and then the driver stays in the same state.*/
    _uart_rx_idle_code(uartp);
  }
  else {
    /* Receiver in active state, a callback is generated, if enabled, after
       a completed transfer.*/
    dmaStreamDisable(uartp->dmarx);
    _uart_rx_complete_isr_code(uartp);
  }
}

/**
 * @brief   TX DMA common service routine.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in] flags     pre-shifted content of the STS register
 */
static void uart_lld_serve_tx_end_irq(UARTDriver *uartp, uint32_t flags) {

  /* DMA errors handling.*/
#if defined(AT32_UART_DMA_ERROR_HOOK)
  if ((flags & AT32_DMA_STS_DTERRF) != 0) {
    AT32_UART_DMA_ERROR_HOOK(uartp);
  }
#else
  (void)flags;
#endif

  dmaStreamDisable(uartp->dmatx);

  /* A callback is generated, if enabled, after a completed transfer.*/
  _uart_tx1_isr_code(uartp);
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if AT32_UART_USE_USART1 || defined(__DOXYGEN__)
#if !defined(AT32_USART1_SUPPRESS_ISR)
#if !defined(AT32_USART1_HANDLER)
#error "AT32_USART1_HANDLER not defined"
#endif
/**
 * @brief   USART1 IRQ handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(AT32_USART1_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  uart_lld_serve_interrupt(&UARTD1);

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif /* AT32_UART_USE_USART1 */

#if AT32_UART_USE_USART2 || defined(__DOXYGEN__)
#if !defined(AT32_USART2_SUPPRESS_ISR)
#if !defined(AT32_USART2_HANDLER)
#error "AT32_USART2_HANDLER not defined"
#endif
/**
 * @brief   USART2 IRQ handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(AT32_USART2_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  uart_lld_serve_interrupt(&UARTD2);

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif /* AT32_UART_USE_USART2 */

#if AT32_UART_USE_USART3 || defined(__DOXYGEN__)
#if !defined(AT32_USART3_SUPPRESS_ISR)
#if !defined(AT32_USART3_HANDLER)
#error "AT32_USART3_HANDLER not defined"
#endif
/**
 * @brief   USART3 IRQ handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(AT32_USART3_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  uart_lld_serve_interrupt(&UARTD3);

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif /* AT32_UART_USE_USART3 */

#if AT32_UART_USE_UART4 || defined(__DOXYGEN__)
#if !defined(AT32_UART4_SUPPRESS_ISR)
#if !defined(AT32_UART4_HANDLER)
#error "AT32_UART4_HANDLER not defined"
#endif
/**
 * @brief   UART4 IRQ handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(AT32_UART4_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  uart_lld_serve_interrupt(&UARTD4);

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif /* AT32_UART_USE_UART4 */

#if AT32_UART_USE_UART5 || defined(__DOXYGEN__)
#if !defined(AT32_UART5_SUPPRESS_ISR)
#if !defined(AT32_UART5_HANDLER)
#error "AT32_UART5_HANDLER not defined"
#endif
/**
 * @brief   UART5 IRQ handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(AT32_UART5_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  uart_lld_serve_interrupt(&UARTD5);

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif /* AT32_UART_USE_UART5 */

#if AT32_UART_USE_USART6 || defined(__DOXYGEN__)
#if !defined(AT32_USART6_SUPPRESS_ISR)
#if !defined(AT32_USART6_HANDLER)
#error "AT32_USART6_HANDLER not defined"
#endif
/**
 * @brief   USART6 IRQ handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(AT32_USART6_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  uart_lld_serve_interrupt(&UARTD6);

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif /* AT32_UART_USE_USART6 */

#if AT32_UART_USE_UART7 || defined(__DOXYGEN__)
#if !defined(AT32_UART7_SUPPRESS_ISR)
#if !defined(AT32_UART7_HANDLER)
#error "AT32_UART7_HANDLER not defined"
#endif
/**
 * @brief   UART7 IRQ handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(AT32_UART7_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  uart_lld_serve_interrupt(&UARTD7);

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif /* AT32_UART_USE_UART7 */

#if AT32_UART_USE_UART8 || defined(__DOXYGEN__)
#if !defined(AT32_UART8_SUPPRESS_ISR)
#if !defined(AT32_UART8_HANDLER)
#error "AT32_UART8_HANDLER not defined"
#endif
/**
 * @brief   UART8 IRQ handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(AT32_UART8_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  uart_lld_serve_interrupt(&UARTD8);

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif /* AT32_UART_USE_UART8 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level UART driver initialization.
 *
 * @notapi
 */
void uart_lld_init(void) {

#if AT32_UART_USE_USART1
  uartObjectInit(&UARTD1);
  UARTD1.usart = USART1;
  UARTD1.clock = AT32_PCLK2;
  UARTD1.dmarxmode = AT32_DMA_CCTRL_DTERRIEN;
  UARTD1.dmatxmode = AT32_DMA_CCTRL_DTERRIEN;
  UARTD1.dmarx = NULL;
  UARTD1.dmatx = NULL;
#if !defined(AT32_USART1_SUPPRESS_ISR) && defined(AT32_USART1_NUMBER)
  nvicEnableVector(AT32_USART1_NUMBER, AT32_UART_USART1_IRQ_PRIORITY);
#endif
#endif

#if AT32_UART_USE_USART2
  uartObjectInit(&UARTD2);
  UARTD2.usart = USART2;
  UARTD2.clock = AT32_PCLK1;
  UARTD2.dmarxmode = AT32_DMA_CCTRL_DTERRIEN;
  UARTD2.dmatxmode = AT32_DMA_CCTRL_DTERRIEN;
  UARTD2.dmarx = NULL;
  UARTD2.dmatx = NULL;
#if !defined(AT32_USART2_SUPPRESS_ISR) && defined(AT32_USART2_NUMBER)
  nvicEnableVector(AT32_USART2_NUMBER, AT32_UART_USART2_IRQ_PRIORITY);
#endif
#endif

#if AT32_UART_USE_USART3
  uartObjectInit(&UARTD3);
  UARTD3.usart = USART3;
  UARTD3.clock = AT32_PCLK1;
  UARTD3.dmarxmode = AT32_DMA_CCTRL_DTERRIEN;
  UARTD3.dmatxmode = AT32_DMA_CCTRL_DTERRIEN;
  UARTD3.dmarx = NULL;
  UARTD3.dmatx = NULL;
#if !defined(AT32_USART3_SUPPRESS_ISR) && defined(AT32_USART3_NUMBER)
  nvicEnableVector(AT32_USART3_NUMBER, AT32_UART_USART3_IRQ_PRIORITY);
#endif
#endif

#if AT32_UART_USE_UART4
  uartObjectInit(&UARTD4);
  UARTD4.usart = UART4;
  UARTD4.clock = AT32_PCLK1;
  UARTD4.dmarxmode = AT32_DMA_CCTRL_DTERRIEN;
  UARTD4.dmatxmode = AT32_DMA_CCTRL_DTERRIEN;
  UARTD4.dmarx = NULL;
  UARTD4.dmatx = NULL;
#if !defined(AT32_UART4_SUPPRESS_ISR) && defined(AT32_UART4_NUMBER)
  nvicEnableVector(AT32_UART4_NUMBER, AT32_UART_UART4_IRQ_PRIORITY);
#endif
#endif

#if AT32_UART_USE_UART5
  uartObjectInit(&UARTD5);
  UARTD5.usart = UART5;
  UARTD5.clock = AT32_PCLK1;
  UARTD5.dmarxmode = AT32_DMA_CCTRL_DTERRIEN;
  UARTD5.dmatxmode = AT32_DMA_CCTRL_DTERRIEN;
  UARTD5.dmarx = NULL;
  UARTD5.dmatx = NULL;
#if !defined(AT32_UART5_SUPPRESS_ISR) && defined(AT32_UART5_NUMBER)
  nvicEnableVector(AT32_UART5_NUMBER, AT32_UART_UART5_IRQ_PRIORITY);
#endif
#endif

#if AT32_UART_USE_USART6
  uartObjectInit(&UARTD6);
  UARTD6.usart = USART6;
  UARTD6.clock = AT32_PCLK2;
  UARTD6.dmarxmode = AT32_DMA_CCTRL_DTERRIEN;
  UARTD6.dmatxmode = AT32_DMA_CCTRL_DTERRIEN;
  UARTD6.dmarx = NULL;
  UARTD6.dmatx = NULL;
#if !defined(AT32_USART6_SUPPRESS_ISR) && defined(AT32_USART6_NUMBER)
  nvicEnableVector(AT32_USART6_NUMBER, AT32_UART_USART6_IRQ_PRIORITY);
#endif
#endif

#if AT32_UART_USE_UART7
  uartObjectInit(&UARTD7);
  UARTD7.usart = UART7;
  UARTD7.clock = AT32_PCLK1;
  UARTD7.dmarxmode = AT32_DMA_CCTRL_DTERRIEN;
  UARTD7.dmatxmode = AT32_DMA_CCTRL_DTERRIEN;
  UARTD7.dmarx = NULL;
  UARTD7.dmatx = NULL;
#if !defined(AT32_UART7_SUPPRESS_ISR) && defined(AT32_UART7_NUMBER)
  nvicEnableVector(AT32_UART7_NUMBER, AT32_UART_UART7_IRQ_PRIORITY);
#endif
#endif

#if AT32_UART_USE_UART8
  uartObjectInit(&UARTD8);
  UARTD8.usart = UART8;
  UARTD8.clock = AT32_PCLK1;
  UARTD8.dmarxmode = AT32_DMA_CCTRL_DTERRIEN;
  UARTD8.dmatxmode = AT32_DMA_CCTRL_DTERRIEN;
  UARTD8.dmarx = NULL;
  UARTD8.dmatx = NULL;
#if !defined(AT32_UART8_SUPPRESS_ISR) && defined(AT32_UART8_NUMBER)
  nvicEnableVector(AT32_UART8_NUMBER, AT32_UART_UART8_IRQ_PRIORITY);
#endif
#endif
}

/**
 * @brief   Configures and activates the UART peripheral.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @notapi
 */
void uart_lld_start(UARTDriver *uartp) {

  if (uartp->state == UART_STOP) {

    if (false) {
    }
#if AT32_UART_USE_USART1
    else if (&UARTD1 == uartp) {
      uartp->dmarx = dmaStreamAllocI(AT32_UART_USART1_RX_DMA_STREAM,
                                     AT32_UART_USART1_IRQ_PRIORITY,
                                     (at32_dmasts_t)uart_lld_serve_rx_end_irq,
                                     (void *)uartp);
      osalDbgAssert(uartp->dmarx != NULL, "unable to allocate stream");
      uartp->dmatx = dmaStreamAllocI(AT32_UART_USART1_TX_DMA_STREAM,
                                     AT32_UART_USART1_IRQ_PRIORITY,
                                     (at32_dmasts_t)uart_lld_serve_tx_end_irq,
                                     (void *)uartp);
      osalDbgAssert(uartp->dmatx != NULL, "unable to allocate stream");

      crmEnableUSART1(true);
      uartp->dmarxmode |= AT32_DMA_CCTRL_CHPL(AT32_UART_USART1_DMA_PRIORITY);
      uartp->dmatxmode |= AT32_DMA_CCTRL_CHPL(AT32_UART_USART1_DMA_PRIORITY);
#if AT32_DMA_SUPPORTS_DMAMUX
      dmaSetRequestSource(uartp->dmarx, AT32_DMAMUX_USART1_RX);
      dmaSetRequestSource(uartp->dmatx, AT32_DMAMUX_USART1_TX);
#endif
    }
#endif

#if AT32_UART_USE_USART2
    else if (&UARTD2 == uartp) {
      uartp->dmarx = dmaStreamAllocI(AT32_UART_USART2_RX_DMA_STREAM,
                                     AT32_UART_USART2_IRQ_PRIORITY,
                                     (at32_dmasts_t)uart_lld_serve_rx_end_irq,
                                     (void *)uartp);
      osalDbgAssert(uartp->dmarx != NULL, "unable to allocate stream");
      uartp->dmatx = dmaStreamAllocI(AT32_UART_USART2_TX_DMA_STREAM,
                                     AT32_UART_USART2_IRQ_PRIORITY,
                                     (at32_dmasts_t)uart_lld_serve_tx_end_irq,
                                     (void *)uartp);
      osalDbgAssert(uartp->dmatx != NULL, "unable to allocate stream");

      crmEnableUSART2(true);
      uartp->dmarxmode |= AT32_DMA_CCTRL_CHPL(AT32_UART_USART2_DMA_PRIORITY);
      uartp->dmatxmode |= AT32_DMA_CCTRL_CHPL(AT32_UART_USART2_DMA_PRIORITY);
#if AT32_DMA_SUPPORTS_DMAMUX
      dmaSetRequestSource(uartp->dmarx, AT32_DMAMUX_USART2_RX);
      dmaSetRequestSource(uartp->dmatx, AT32_DMAMUX_USART2_TX);
#endif
    }
#endif

#if AT32_UART_USE_USART3
    else if (&UARTD3 == uartp) {
      uartp->dmarx = dmaStreamAllocI(AT32_UART_USART3_RX_DMA_STREAM,
                                     AT32_UART_USART3_IRQ_PRIORITY,
                                     (at32_dmasts_t)uart_lld_serve_rx_end_irq,
                                     (void *)uartp);
      osalDbgAssert(uartp->dmarx != NULL, "unable to allocate stream");
      uartp->dmatx = dmaStreamAllocI(AT32_UART_USART3_TX_DMA_STREAM,
                                     AT32_UART_USART3_IRQ_PRIORITY,
                                     (at32_dmasts_t)uart_lld_serve_tx_end_irq,
                                     (void *)uartp);
      osalDbgAssert(uartp->dmatx != NULL, "unable to allocate stream");

      crmEnableUSART3(true);
      uartp->dmarxmode |= AT32_DMA_CCTRL_CHPL(AT32_UART_USART3_DMA_PRIORITY);
      uartp->dmatxmode |= AT32_DMA_CCTRL_CHPL(AT32_UART_USART3_DMA_PRIORITY);
#if AT32_DMA_SUPPORTS_DMAMUX
      dmaSetRequestSource(uartp->dmarx, AT32_DMAMUX_USART3_RX);
      dmaSetRequestSource(uartp->dmatx, AT32_DMAMUX_USART3_TX);
#endif
    }
#endif

#if AT32_UART_USE_UART4
    else if (&UARTD4 == uartp) {
      uartp->dmarx = dmaStreamAllocI(AT32_UART_UART4_RX_DMA_STREAM,
                                     AT32_UART_UART4_IRQ_PRIORITY,
                                     (at32_dmasts_t)uart_lld_serve_rx_end_irq,
                                     (void *)uartp);
      osalDbgAssert(uartp->dmarx != NULL, "unable to allocate stream");
      uartp->dmatx = dmaStreamAllocI(AT32_UART_UART4_TX_DMA_STREAM,
                                     AT32_UART_UART4_IRQ_PRIORITY,
                                     (at32_dmasts_t)uart_lld_serve_tx_end_irq,
                                     (void *)uartp);
      osalDbgAssert(uartp->dmatx != NULL, "unable to allocate stream");

      crmEnableUART4(true);
      uartp->dmarxmode |= AT32_DMA_CCTRL_CHPL(AT32_UART_UART4_DMA_PRIORITY);
      uartp->dmatxmode |= AT32_DMA_CCTRL_CHPL(AT32_UART_UART4_DMA_PRIORITY);
#if AT32_DMA_SUPPORTS_DMAMUX
      dmaSetRequestSource(uartp->dmarx, AT32_DMAMUX_UART4_RX);
      dmaSetRequestSource(uartp->dmatx, AT32_DMAMUX_UART4_TX);
#endif
    }
#endif

#if AT32_UART_USE_UART5
    else if (&UARTD5 == uartp) {
      uartp->dmarx = dmaStreamAllocI(AT32_UART_UART5_RX_DMA_STREAM,
                                     AT32_UART_UART5_IRQ_PRIORITY,
                                     (at32_dmasts_t)uart_lld_serve_rx_end_irq,
                                     (void *)uartp);
      osalDbgAssert(uartp->dmarx != NULL, "unable to allocate stream");
      uartp->dmatx = dmaStreamAllocI(AT32_UART_UART5_TX_DMA_STREAM,
                                     AT32_UART_UART5_IRQ_PRIORITY,
                                     (at32_dmasts_t)uart_lld_serve_tx_end_irq,
                                     (void *)uartp);
      osalDbgAssert(uartp->dmatx != NULL, "unable to allocate stream");

      crmEnableUART5(true);
      uartp->dmarxmode |= AT32_DMA_CCTRL_CHPL(AT32_UART_UART5_DMA_PRIORITY);
      uartp->dmatxmode |= AT32_DMA_CCTRL_CHPL(AT32_UART_UART5_DMA_PRIORITY);
#if AT32_DMA_SUPPORTS_DMAMUX
      dmaSetRequestSource(uartp->dmarx, AT32_DMAMUX_UART5_RX);
      dmaSetRequestSource(uartp->dmatx, AT32_DMAMUX_UART5_TX);
#endif
    }
#endif

#if AT32_UART_USE_USART6
    else if (&UARTD6 == uartp) {
      uartp->dmarx = dmaStreamAllocI(AT32_UART_USART6_RX_DMA_STREAM,
                                     AT32_UART_USART6_IRQ_PRIORITY,
                                     (at32_dmasts_t)uart_lld_serve_rx_end_irq,
                                     (void *)uartp);
      osalDbgAssert(uartp->dmarx != NULL, "unable to allocate stream");
      uartp->dmatx = dmaStreamAllocI(AT32_UART_USART6_TX_DMA_STREAM,
                                     AT32_UART_USART6_IRQ_PRIORITY,
                                     (at32_dmasts_t)uart_lld_serve_tx_end_irq,
                                     (void *)uartp);
      osalDbgAssert(uartp->dmatx != NULL, "unable to allocate stream");

      crmEnableUSART6(true);
      uartp->dmarxmode |= AT32_DMA_CCTRL_CHPL(AT32_UART_USART6_DMA_PRIORITY);
      uartp->dmatxmode |= AT32_DMA_CCTRL_CHPL(AT32_UART_USART6_DMA_PRIORITY);
#if AT32_DMA_SUPPORTS_DMAMUX
      dmaSetRequestSource(uartp->dmarx, AT32_DMAMUX_USART6_RX);
      dmaSetRequestSource(uartp->dmatx, AT32_DMAMUX_USART6_TX);
#endif
    }
#endif

#if AT32_UART_USE_UART7
    else if (&UARTD7 == uartp) {
      uartp->dmarx = dmaStreamAllocI(AT32_UART_UART7_RX_DMA_STREAM,
                                     AT32_UART_UART7_IRQ_PRIORITY,
                                     (at32_dmasts_t)uart_lld_serve_rx_end_irq,
                                     (void *)uartp);
      osalDbgAssert(uartp->dmarx != NULL, "unable to allocate stream");
      uartp->dmatx = dmaStreamAllocI(AT32_UART_UART7_TX_DMA_STREAM,
                                     AT32_UART_UART7_IRQ_PRIORITY,
                                     (at32_dmasts_t)uart_lld_serve_tx_end_irq,
                                     (void *)uartp);
      osalDbgAssert(uartp->dmatx != NULL, "unable to allocate stream");

      crmEnableUART7(true);
      uartp->dmarxmode |= AT32_DMA_CCTRL_CHPL(AT32_UART_UART7_DMA_PRIORITY);
      uartp->dmatxmode |= AT32_DMA_CCTRL_CHPL(AT32_UART_UART7_DMA_PRIORITY);
#if AT32_DMA_SUPPORTS_DMAMUX
      dmaSetRequestSource(uartp->dmarx, AT32_DMAMUX_UART7_RX);
      dmaSetRequestSource(uartp->dmatx, AT32_DMAMUX_UART7_TX);
#endif
    }
#endif

#if AT32_UART_USE_UART8
    else if (&UARTD8 == uartp) {
      uartp->dmarx = dmaStreamAllocI(AT32_UART_UART8_RX_DMA_STREAM,
                                     AT32_UART_UART8_IRQ_PRIORITY,
                                     (at32_dmasts_t)uart_lld_serve_rx_end_irq,
                                     (void *)uartp);
      osalDbgAssert(uartp->dmarx != NULL, "unable to allocate stream");
      uartp->dmatx = dmaStreamAllocI(AT32_UART_UART8_TX_DMA_STREAM,
                                     AT32_UART_UART8_IRQ_PRIORITY,
                                     (at32_dmasts_t)uart_lld_serve_tx_end_irq,
                                     (void *)uartp);
      osalDbgAssert(uartp->dmatx != NULL, "unable to allocate stream");

      crmEnableUART8(true);
      uartp->dmarxmode |= AT32_DMA_CCTRL_CHPL(AT32_UART_UART8_DMA_PRIORITY);
      uartp->dmatxmode |= AT32_DMA_CCTRL_CHPL(AT32_UART_UART8_DMA_PRIORITY);
#if AT32_DMA_SUPPORTS_DMAMUX
      dmaSetRequestSource(uartp->dmarx, AT32_DMAMUX_UART8_RX);
      dmaSetRequestSource(uartp->dmatx, AT32_DMAMUX_UART8_TX);
#endif
    }
#endif
    else {
      osalDbgAssert(false, "invalid USART instance");
    }

    /* Static DMA setup, the transfer size depends on the USART settings,
       it is 16 bits if DBN=1 and PEN=0 else it is 8 bits.*/
    if ((uartp->config->ctrl1 & (USART_CTRL1_DBN0 | USART_CTRL1_PEN)) == USART_CTRL1_DBN0) {
      uartp->dmarxmode |= AT32_DMA_CCTRL_PWIDTH_HWORD | AT32_DMA_CCTRL_MWIDTH_HWORD;
      uartp->dmatxmode |= AT32_DMA_CCTRL_PWIDTH_HWORD | AT32_DMA_CCTRL_MWIDTH_HWORD;
    }
    dmaStreamSetPeripheral(uartp->dmarx, &uartp->usart->DT);
    dmaStreamSetPeripheral(uartp->dmatx, &uartp->usart->DT);
    uartp->rxbuf = 0;
  }

  uartp->rxstate = UART_RX_IDLE;
  uartp->txstate = UART_TX_IDLE;
  usart_start(uartp);
}

/**
 * @brief   Deactivates the UART peripheral.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @notapi
 */
void uart_lld_stop(UARTDriver *uartp) {

  if (uartp->state == UART_READY) {
    usart_stop(uartp);
    dmaStreamFreeI(uartp->dmarx);
    dmaStreamFreeI(uartp->dmatx);
    uartp->dmarx = NULL;
    uartp->dmatx = NULL;

#if AT32_UART_USE_USART1
    if (&UARTD1 == uartp) {
      crmDisableUSART1();
      return;
    }
#endif

#if AT32_UART_USE_USART2
    if (&UARTD2 == uartp) {
      crmDisableUSART2();
      return;
    }
#endif

#if AT32_UART_USE_USART3
    if (&UARTD3 == uartp) {
      crmDisableUSART3();
      return;
    }
#endif

#if AT32_UART_USE_UART4
    if (&UARTD4 == uartp) {
      crmDisableUART4();
      return;
    }
#endif

#if AT32_UART_USE_UART5
    if (&UARTD5 == uartp) {
      crmDisableUART5();
      return;
    }
#endif

#if AT32_UART_USE_USART6
    if (&UARTD6 == uartp) {
      crmDisableUSART6();
      return;
    }
#endif

#if AT32_UART_USE_UART7
    if (&UARTD7 == uartp) {
      crmDisableUART7();
      return;
    }
#endif

#if AT32_UART_USE_UART8
    if (&UARTD8 == uartp) {
      crmDisableUART8();
      return;
    }
#endif
  }
}

/**
 * @brief   Starts a transmission on the UART peripheral.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in] n         number of data frames to send
 * @param[in] txbuf     the pointer to the transmit buffer
 *
 * @notapi
 */
void uart_lld_start_send(UARTDriver *uartp, size_t n, const void *txbuf) {

  /* TX DMA channel preparation.*/
  dmaStreamSetMemory0(uartp->dmatx, txbuf);
  dmaStreamSetTransactionSize(uartp->dmatx, n);
  dmaStreamSetMode(uartp->dmatx, uartp->dmatxmode     | AT32_DMA_CCTRL_DTD_M2P |
                                 AT32_DMA_CCTRL_MINCM | AT32_DMA_CCTRL_FDTIEN);

  /* Only enable TDC interrupt if there's a callback attached to it or
     if called from uartSendFullTimeout(). Also we need to clear TDC flag
     which could be set before.*/
#if UART_USE_WAIT == TRUE
  if ((uartp->config->txend2_cb != NULL) || (uartp->early == false)) {
#else
  if (uartp->config->txend2_cb != NULL) {
#endif
    uartp->usart->CTRL1 |= USART_CTRL1_TDCIEN;
  }

  /* Starting transfer.*/
  dmaStreamEnable(uartp->dmatx);
}

/**
 * @brief   Stops any ongoing transmission.
 * @note    Stopping a transmission also suppresses the transmission callbacks.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @return              The number of data frames not transmitted by the
 *                      stopped transmit operation.
 *
 * @notapi
 */
size_t uart_lld_stop_send(UARTDriver *uartp) {

  dmaStreamDisable(uartp->dmatx);

  return dmaStreamGetTransactionSize(uartp->dmatx);
}

/**
 * @brief   Starts a receive operation on the UART peripheral.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in] n         number of data frames to send
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
void uart_lld_start_receive(UARTDriver *uartp, size_t n, void *rxbuf) {

  /* Stopping previous activity (idle state).*/
  dmaStreamDisable(uartp->dmarx);

  /* RX DMA channel preparation.*/
  dmaStreamSetMemory0(uartp->dmarx, rxbuf);
  dmaStreamSetTransactionSize(uartp->dmarx, n);
  dmaStreamSetMode(uartp->dmarx, uartp->dmarxmode     | AT32_DMA_CCTRL_DTD_P2M |
                                 AT32_DMA_CCTRL_MINCM | AT32_DMA_CCTRL_FDTIEN);

  /* Starting transfer.*/
  dmaStreamEnable(uartp->dmarx);
}

/**
 * @brief   Stops any ongoing receive operation.
 * @note    Stopping a receive operation also suppresses the receive callbacks.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @return              The number of data frames not received by the
 *                      stopped receive operation.
 *
 * @notapi
 */
size_t uart_lld_stop_receive(UARTDriver *uartp) {
  size_t n;

  dmaStreamDisable(uartp->dmarx);
  n = dmaStreamGetTransactionSize(uartp->dmarx);
  uart_enter_rx_idle_loop(uartp);

  return n;
}

/**
 * @brief   USART common service routine.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
void uart_lld_serve_interrupt(UARTDriver *uartp) {
  uint32_t sts;
  USART_TypeDef *u = uartp->usart;
  uint32_t ctrl1 = u->CTRL1;

  /* Reading and clearing status.*/
  sts = u->STS;
  #if !defined (AT32F435_437)
  u->IFC = sts;
  #endif

  if (sts & (USART_STS_BFF  | USART_STS_ROERR | USART_STS_NERR |
             USART_STS_FERR | USART_STS_PERR)) {
    _uart_rx_error_isr_code(uartp, translate_errors(sts));
  }

  if ((sts & USART_STS_TDC) && (ctrl1 & USART_CTRL1_TDCIEN)) {
    /* TDC interrupt disabled.*/
    u->CTRL1 = ctrl1 & ~USART_CTRL1_TDCIEN;

    /* End of transmission, a callback is generated.*/
    _uart_tx2_isr_code(uartp);
  }

  /* Timeout interrupt sources are only checked if enabled in CTRL1.*/
  if (((ctrl1 & USART_CTRL1_IDLEIEN) && (sts & USART_STS_IDLEF)) ||
      ((ctrl1 & USART_CTRL1_RETODIE) && (sts & USART_STS_RTODF))) {
    _uart_timeout_isr_code(uartp);
  }
}

#endif /* HAL_USE_UART */

/** @} */
