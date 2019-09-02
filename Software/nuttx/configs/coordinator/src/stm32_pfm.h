/************************************************************************************
 * configs/coordinator/stm32_pfm.h
 *
 *   Copyright (C) 2019 Uros Platise. All rights reserved.
 *   Author: Akrapong Patchararungruang (akrapong.p@ku.th)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __CONFIG_BOARD_COORDINATOR_STM32_PFM_H
#define __CONFIG_BOARD_COORDINATOR_STM32_PFM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Helpers **************************************************************************/

#define STM32_PFM_START(d,leading,cycle) ((d)->ops->start(d,leading,cycle))
#define STM32_PFM_STOP(d)                ((d)->ops->stop(d))

/* Some constants
 *
 */
#define CONFIG_STM32_PFM_FREQ    200000  /* Input frequency of the timer */
#define CONFIG_STM32_PFM_SHOT_PEROID  2
/************************************************************************************
 * Public Types
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

struct stm32_pfm_dev_s
{
  FAR struct stm32_pfm_ops_s    *ops;
};

struct stm32_pfm_ops_s
{
  void (*start)(FAR struct stm32_pfm_dev_s *dev, uint16_t leading_period, uint16_t total_cycle);
  void (*stop)(FAR struct stm32_pfm_dev_s *dev);
  bool (*is_idle)(FAR struct stm32_pfm_dev_s *dev);
};

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/****************************************************************************
 * Name: stm32_pfm_init
 *
 * Description:
 *   Initialize PFM drivers
 *
 ****************************************************************************/

FAR struct stm32_pfm_dev_s *stm32_pfm_init(unsigned int motor_id);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIG_BOARD_COORDINATOR_STM32_PFM_H */
