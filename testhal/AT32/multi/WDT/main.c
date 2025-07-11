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

#include "ch.h"
#include "hal.h"

#include "portab.h"

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Board-dependent setup code.
   */
  portab_setup();

  /*
   * Starting the watchdog driver.
   */
  wdgStart(&PORTAB_WDT1, &wdgcfg);

  /*
   * Normal main() thread activity, it resets the watchdog.
   */
  while (true) {
    wdgReset(&PORTAB_WDT1);
    palToggleLine(PORTAB_BLINK_LED1);
    palToggleLine(PORTAB_BLINK_LED2);
    palToggleLine(PORTAB_BLINK_LED3);
    chThdSleepMilliseconds(500);
  }
  return 0;
}
