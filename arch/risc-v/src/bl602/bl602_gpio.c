/****************************************************************************
 * arch/risc-v/src/bl602/bl602_gpio.c
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

#include <stdint.h>
#include "hardware/bl602_gpio.h"
#include "hardware/bl602_glb.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: <Static function name>
 *
 * Description:
 *   Description of the operation of the static function.
 *
 * Input Parameters:
 *   A list of input parameters, one-per-line, appears here along with a
 *   description of each input parameter.
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   Anything else that one might need to know to use this function.
 *
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gpio_init
 *
 * Description:
 *   Init a gpio pin.
 *
 * Input Parameters:
 *   cfg: gpio configuration
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   Anything else that one might need to know to use this function.
 *
 ****************************************************************************/

void gpio_init(struct gpio_cfg_s *cfg)
{
  uint8_t   gpio_pin = cfg->gpio_pin;
  uint32_t *p_out;
  uint32_t  pos;
  uint32_t  tmp_out;
  uint32_t  tmp_val;

  p_out   = (uint32_t *)(GLB_BASE + GLB_GPIO_OUTPUT_EN_OFFSET +
                       ((gpio_pin >> 5) << 2));
  pos     = gpio_pin % 32;
  tmp_out = *p_out;

  /* Disable output anyway */

  tmp_out &= (~(1 << pos));
  *p_out = tmp_out;

  tmp_val = BL_RD_WORD(GLB_BASE + GLB_GPIO_OFFSET + gpio_pin / 2 * 4);

  if (gpio_pin % 2 == 0)
    {
      /* Set input or output */

      if (cfg->gpio_mode == GPIO_MODE_OUTPUT)
        {
          tmp_val = BL_CLR_REG_BIT(tmp_val, GLB_REG_GPIO_0_IE);
          tmp_out |= (1 << pos);
        }
      else
        {
          tmp_val = BL_SET_REG_BIT(tmp_val, GLB_REG_GPIO_0_IE);
        }

      /* Set pull up or down */

      tmp_val = BL_CLR_REG_BIT(tmp_val, GLB_REG_GPIO_0_PU);
      tmp_val = BL_CLR_REG_BIT(tmp_val, GLB_REG_GPIO_0_PD);
      if (cfg->pull_type == GPIO_PULL_UP)
        {
          tmp_val = BL_SET_REG_BIT(tmp_val, GLB_REG_GPIO_0_PU);
        }
      else if (cfg->pull_type == GPIO_PULL_DOWN)
        {
          tmp_val = BL_SET_REG_BIT(tmp_val, GLB_REG_GPIO_0_PD);
        }

      tmp_val = BL_SET_REG_BITS_VAL(tmp_val, GLB_REG_GPIO_0_DRV, cfg->drive);
      tmp_val =
        BL_SET_REG_BITS_VAL(tmp_val, GLB_REG_GPIO_0_SMT, cfg->smt_ctrl);
      tmp_val =
        BL_SET_REG_BITS_VAL(tmp_val, GLB_REG_GPIO_0_FUNC_SEL, cfg->gpio_fun);
    }
  else
    {
      /* Set input or output */

      if (cfg->gpio_mode == GPIO_MODE_OUTPUT)
        {
          tmp_val = BL_CLR_REG_BIT(tmp_val, GLB_REG_GPIO_1_IE);
          tmp_out |= (1 << pos);
        }
      else
        {
          tmp_val = BL_SET_REG_BIT(tmp_val, GLB_REG_GPIO_1_IE);
        }

      /* Set pull up or down */

      tmp_val = BL_CLR_REG_BIT(tmp_val, GLB_REG_GPIO_1_PU);
      tmp_val = BL_CLR_REG_BIT(tmp_val, GLB_REG_GPIO_1_PD);
      if (cfg->pull_type == GPIO_PULL_UP)
        {
          tmp_val = BL_SET_REG_BIT(tmp_val, GLB_REG_GPIO_1_PU);
        }
      else if (cfg->pull_type == GPIO_PULL_DOWN)
        {
          tmp_val = BL_SET_REG_BIT(tmp_val, GLB_REG_GPIO_1_PD);
        }

      tmp_val = BL_SET_REG_BITS_VAL(tmp_val, GLB_REG_GPIO_1_DRV, cfg->drive);
      tmp_val =
        BL_SET_REG_BITS_VAL(tmp_val, GLB_REG_GPIO_1_SMT, cfg->smt_ctrl);
      tmp_val =
        BL_SET_REG_BITS_VAL(tmp_val, GLB_REG_GPIO_1_FUNC_SEL, cfg->gpio_fun);
    }

  BL_WR_WORD(GLB_BASE + GLB_GPIO_OFFSET + gpio_pin / 2 * 4, tmp_val);

  *p_out = tmp_out;
}
