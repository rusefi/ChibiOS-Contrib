/*
    ChibiOS/RT - Copyright (C) 2006-2014 Giovanni Di Sirio.
                           (C) 2015 RedoX (https://github.com/RedoXyde)
                           (C) 2023-2025 HorrorTroll (https://github.com/HorrorTroll)
                           (C) 2023-2025 Zhaqian (https://github.com/zhaqian12)
                           (C) 2024-2025 Maxjta (https://github.com/Maxjta)

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file        AT32F405/cmparams.h
 * @brief       ARM Cortex-M4F parameters for the Artery AT32F405
 *
 * @defgroup    ARMCMx_AT32F405 Artery AT32F405 Specific Parameters
 * @ingroup     ARMCMx_SPECIFIC
 * @details     This file contains the Cortex-M4F specific parameters for the
 *              Artery AT32F405 platform.
 * @{
 */

#ifndef _CMPARAMS_H_
#define _CMPARAMS_H_

/**
 * @brief   Cortex core model.
 */
#define CORTEX_MODEL            4

/**
 * @brief   Systick unit presence.
 */
#define CORTEX_HAS_ST           TRUE

/**
 * @brief   Floating Point unit presence.
 */
#define CORTEX_HAS_FPU          TRUE

/**
 * @brief   Number of bits in priority masks.
 */
#define CORTEX_PRIORITY_BITS    4

/* If the device type is not externally defined, for example from the Makefile,
   then a file named board.h is included. This file must contain a device
   definition compatible with the include file.*/
#if !defined(AT32F405KB) && !defined(AT32F405KC) && !defined(AT32F405CB) && \
    !defined(AT32F405CC) && !defined(AT32F405RB) && !defined(AT32F405RC)
#include "board.h"
#endif

/**
 * @brief   Number of interrupt vectors.
 * @note    This number does not include the 16 system vectors and must be
 *          rounded to a multiple of 8.
 */
#define CORTEX_NUM_VECTORS      104

/* The following code is not processed when the file is included from an
   asm module.*/
#if !defined(_FROM_ASM_)

/* Including the device CMSIS header. Note, we are not using the definitions
   from this header because we need this file to be usable also from
   assembler source files. We verify that the info matches instead.*/
#include "at32f402_405.h"

/*lint -save -e9029 [10.4] Signedness comes from external files, it is
  unpredictable but gives no problems.*/
#if CORTEX_MODEL != __CORTEX_M
#error "CMSIS __CORTEX_M mismatch"
#endif

#if CORTEX_PRIORITY_BITS != __NVIC_PRIO_BITS
#error "CMSIS __NVIC_PRIO_BITS mismatch"
#endif
/*lint -restore*/

#endif /* !defined(_FROM_ASM_) */

#endif /* _CMPARAMS_H_ */

/** @} */
