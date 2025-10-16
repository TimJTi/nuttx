/****************************************************************************
 * arch/arm/src/sama5/sam_idle.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/board/board.h>
#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/power/pm.h>

#include <nuttx/irq.h>

#include "arm_internal.h"
#include "hardware/sam_pmc.h"
#include "chip.h"
#include "sam_periphclks.h"
#include "sam_pio.h"
#include <board.h>

#include <nuttx/power/act8945a.h>
#include <nuttx/power/regulator.h>
#include <nuttx/power/consumer.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Does the board support an IDLE LED to indicate that the board is in the
 * IDLE state?
 */

#if defined(CONFIG_ARCH_LEDS) && defined(LED_IDLE)
#  define BEGIN_IDLE() board_autoled_on(LED_IDLE)
#  define END_IDLE()   board_autoled_off(LED_IDLE)
#else
#  define BEGIN_IDLE()
#  define END_IDLE()
#endif

#define MPDDRC_LPR_LPCB_SELFREFRESH 0x01
#define SAM_MPDDRC_LPR              0xf000c01c//(SAM_MPDDRC_VBASE + 0xc01c)

#define PIO_LCD_BACKLIGHT_ENABLE (PIO_OUTPUT | PIO_CFG_DEFAULT | \
                                  PIO_PORT_PIOD | PIO_PIN8 | PIO_OUTPUT_SET)
/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void sam_pmstandby(void)
{
  //ULP0 mode
  uint32_t regval;
  uint32_t read_reg[4];
  const struct regulator_desc_s *act8945a_desc;
  struct regulator_s            *act8945a;
  unsigned int                  enabled;
  unsigned int                  selector;

  sam_piowrite(PIO_LCD_BACKLIGHT_ENABLE, true);

  act8945a = regulator_get(CONFIG_ACT8945A_DCDC3_NAME);
  regulator_disable(act8945a);

  read_reg[0] = getreg32(SAM_PMC_PCSR0);
  read_reg[1] = getreg32(SAM_PMC_PCSR1);
  read_reg[2] = getreg32(SAM_PMC_SCSR);
  read_reg[3] = getreg32(SAM_PMC_CKGR_UCKR);

  regval = getreg32(SAM_MPDDRC_LPR);
  regval &= ~0x03;
  regval |= MPDDRC_LPR_LPCB_SELFREFRESH;
  putreg32(regval, SAM_MPDDRC_LPR);

  sam_mpddrc_disableclk();
  sam_uhphs_disableclk();
  sam_udphs_disableclk();
  sam_mcan0_disableclk();
  sam_mcan1_disableclk();
  sam_classd_disableclk();
  sam_adc_disableclk();
  sam_qspi0_disableclk();
  sam_spi0_disableclk();
  sam_spi1_disableclk();
  sam_pwm_disableclk();
  sam_lcdc_disableclk();
  sam_pio_disableclk();
  sam_flexcom2_disableclk();
  sam_flexcom3_disableclk();
  sam_flexcom4_disableclk();
  sam_uart3_disableclk();
  sam_twi0_disableclk();
  sam_twi1_disableclk();
  sam_piob_disableclk();
  sam_pioc_disableclk();
  sam_piod_disableclk();
  sam_xdmac0_disableclk();
  sam_xdmac1_disableclk();

  int i;
  for (i = 0; i < 32; i++)
    {
      sam_disableperiph0(i);
    }
  for (i = 32; i < 79; i++)
    {
      sam_disableperiph0(i - 32);
    }

  regval = getreg32(SAM_PMC_MCKR);
  regval &= ~0x03;;
  regval |= 0;
  putreg32(regval, SAM_PMC_MCKR);


  asm("cpsid if");
  asm("WFI");

  while(1);

}

/****************************************************************************
 * Name: up_idlepm
 *
 * Description:
 *   Perform IDLE state power management.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static void up_idlepm(void)
{
  static enum pm_state_e oldstate = PM_NORMAL;
  enum pm_state_e newstate;
  irqstate_t flags;
  int ret;

  /* Decide, which power saving level can be obtained */

  newstate = pm_checkstate(PM_IDLE_DOMAIN);

  /* Check for state changes */

  if (newstate != oldstate)
    {
      flags = enter_critical_section();

      /* Perform board-specific, state-dependent logic here */

      _info("newstate= %d oldstate=%d\n", newstate, oldstate);

      /* Then force the global state change */

      ret = pm_changestate(PM_IDLE_DOMAIN, newstate);
      if (ret < 0)
        {
          /* The new state change failed, revert to the preceding state */

          pm_changestate(PM_IDLE_DOMAIN, oldstate);
        }
      else
        {
          /* Save the new state */

          oldstate = newstate;
        }

      /* MCU-specific power management logic */

      switch (newstate)
        {
        case PM_NORMAL:
          break;

        case PM_IDLE:
          break;

        case PM_STANDBY:
          //sam_pmstop(true);
          break;

        case PM_SLEEP:
          //sam_pmstandby();
          break;

        default:
          break;
        }

      leave_critical_section(flags);
    }
}
#else
#  define up_idlepm()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_idle
 *
 * Description:
 *   up_idle() is the logic that will be executed when there is no other
 *   ready-to-run task.  This is processor idle time and will continue until
 *   some interrupt occurs to cause a context switch from the idle task.
 *
 *   Processing in this state may be processor-specific. e.g., this is where
 *   power management operations might be performed.
 *
 ****************************************************************************/

void up_idle(void)
{
#if defined(CONFIG_SUPPRESS_INTERRUPTS) || defined(CONFIG_SUPPRESS_TIMER_INTS)
  /* If the system is idle and there are no timer interrupts, then process
   * "fake" timer interrupts. Hopefully, something will wake up.
   */

  nxsched_process_timer();
#else

  /* Perform IDLE mode power management */

  up_idlepm();

  /* Sleep until an interrupt occurs to save power. */

  BEGIN_IDLE();
  //asm("WFI");
  END_IDLE();
#endif
}
