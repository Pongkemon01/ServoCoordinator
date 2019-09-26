/************************************************************************************
 * configs/coordinator/stm32_pfm.c
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_internal.h"
#include "up_arch.h"
#include "stm32_tim.h"

#include "stm32_gpio.h"
#include "stm32_pfm.h"

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* Configuration ********************************************************************/

/* Timer devices may be used for different purposes.  Such special purposes include:
 *
 * - To generate modulated outputs for such things as motor control.  If
 *   CONFIG_STM32F7_TIMn is defined then the CONFIG_STM32F7_TIMn_PWM may also be
 *   defined to indicate that the timer is intended to be used for pulsed output
 *   modulation.
 *
 * - To control periodic ADC input sampling.  If CONFIG_STM32F7_TIMn is defined then
 *   CONFIG_STM32F7_TIMn_ADC may also be defined to indicate that timer "n" is
 *   intended to be used for that purpose.
 *
 * - To control periodic DAC outputs.  If CONFIG_STM32F7_TIMn is defined then
 *   CONFIG_STM32F7_TIMn_DAC may also be defined to indicate that timer "n" is
 *   intended to be used for that purpose.
 *
 * - To use a Quadrature Encoder.  If CONFIG_STM32F7_TIMn is defined then
 *   CONFIG_STM32F7_TIMn_QE may also be defined to indicate that timer "n" is
 *   intended to be used for that purpose.
 *
 * - To use for other purposes. If CONFIG_STM32F7_TIMn is defined then the
 *   timer "n" is cofigured to operates for other pursose.
 *
 * In any of these cases, the timer will not be used by this timer module.
 */
#if defined(CONFIG_STM32F7_PFM)
#define CONFIG_STM32F7_PFM1
#define CONFIG_STM32F7_PFM2
#define CONFIG_STM32F7_PFM3
#define CONFIG_STM32F7_PFM4
#define CONFIG_STM32F7_PFM5
#define CONFIG_STM32F7_PFM6
#endif

#if defined(CONFIG_STM32F7_TIM9_PWM) || defined (CONFIG_STM32F7_TIM9_ADC) || \
    defined(CONFIG_STM32F7_TIM9_DAC) || defined(CONFIG_STM32F7_TIM9_QE) || \
    !defined(CONFIG_STM32F7_TIM9)
#  undef CONFIG_STM32F7_PFM1
#endif
#if defined(CONFIG_STM32F7_TIM10_PWM) || defined (CONFIG_STM32F7_TIM10_ADC) || \
    defined(CONFIG_STM32F7_TIM10_DAC) || defined(CONFIG_STM32F7_TIM10_QE) || \
    !defined(CONFIG_STM32F7_TIM10)
#  undef CONFIG_STM32F7_PFM2
#endif
#if defined(CONFIG_STM32F7_TIM11_PWM) || defined (CONFIG_STM32F7_TIM11_ADC) || \
    defined(CONFIG_STM32F7_TIM11_DAC) || defined(CONFIG_STM32F7_TIM11_QE) || \
    !defined(CONFIG_STM32F7_TIM11)
#  undef CONFIG_STM32F7_PFM3
#endif
#if defined(CONFIG_STM32F7_TIM12_PWM) || defined (CONFIG_STM32F7_TIM12_ADC) || \
    defined(CONFIG_STM32F7_TIM12_DAC) || defined(CONFIG_STM32F7_TIM12_QE) || \
    !defined(CONFIG_STM32F7_TIM12)
#  undef CONFIG_STM32F7_PFM4
#endif
#if defined(CONFIG_STM32F7_TIM13_PWM) || defined (CONFIG_STM32F7_TIM13_ADC) || \
    defined(CONFIG_STM32F7_TIM13_DAC) || defined(CONFIG_STM32F7_TIM13_QE) || \
    !defined(CONFIG_STM32F7_TIM13)
#  undef CONFIG_STM32F7_PFM5
#endif
#if defined(CONFIG_STM32F7_TIM14_PWM) || defined (CONFIG_STM32F7_TIM14_ADC) || \
    defined(CONFIG_STM32F7_TIM14_DAC) || defined(CONFIG_STM32F7_TIM14_QE) || \
    !defined(CONFIG_STM32F7_TIM14)
#  undef CONFIG_STM32F7_PFM6
#endif

/* This module then only compiles if there are enabled timers that are not
 * intended for some other purpose.
 */

#if defined(CONFIG_STM32F7_PFM1)  || defined(CONFIG_STM32F7_PFM2)  || \
    defined(CONFIG_STM32F7_PFM3)  || defined(CONFIG_STM32F7_PFM4)  || \
    defined(CONFIG_STM32F7_PFM5)  || defined(CONFIG_STM32F7_PFM6)


/************************************************************************************
 * Private Types
 ************************************************************************************/
/* States of the PFM device */
typedef enum
{
  STM32_PFM_STATE_IDLE,
  STM32_PFM_STATE_LEADING,
  STM32_PFM_STATE_SHOT,
}stm32_pfm_state_t;

/* PFM Device Structure */

struct stm32_pfm_priv_s
{
  struct stm32_pfm_ops_s  *ops;
  struct stm32_tim_dev_s  *tim;
  int                     timer_id;
  uint8_t                 compare_channel;

  /* Operation variables */
  volatile stm32_pfm_state_t  state;
  volatile uint16_t           shadow_ccr;
  volatile uint16_t           current_leading_period;
  volatile uint32_t           cycle_count;
};

/* Prototypes */
static void stm32_pfm_start(FAR struct stm32_pfm_dev_s *dev, 
            uint16_t leading_period, uint16_t total_cycle);
static void stm32_pfm_stop(FAR struct stm32_pfm_dev_s *dev);
static bool stm32_pfm_is_idle(FAR struct stm32_pfm_dev_s *dev);

/************************************************************************************
 * Private Functions
 ************************************************************************************/


/************************************************************************************
 * Core Functions and Data
 ************************************************************************************/
struct stm32_pfm_ops_s stm32_pfm_ops =
{
  .start    = stm32_pfm_start,
  .stop     = stm32_pfm_stop,
  .is_idle  = stm32_pfm_is_idle
};

#ifdef CONFIG_STM32F7_PFM1
struct stm32_pfm_priv_s stm32_pfm1_priv =
{
  .ops                    = &stm32_pfm_ops,
  .tim                    = 0,
  .timer_id               = 9,
  .compare_channel        = 1,
  .state                  = STM32_PFM_STATE_IDLE,
  .shadow_ccr             = 0,
  .current_leading_period = 0,
  .cycle_count            = 0
};
#endif

#ifdef CONFIG_STM32F7_PFM2
struct stm32_pfm_priv_s stm32_pfm2_priv =
{
  .ops                    = &stm32_pfm_ops,
  .tim                    = 0,
  .timer_id               = 10,
  .compare_channel        = 1,
  .state                  = STM32_PFM_STATE_IDLE,
  .shadow_ccr             = 0,
  .current_leading_period = 0,
  .cycle_count            = 0
};
#endif

#ifdef CONFIG_STM32F7_PFM3
struct stm32_pfm_priv_s stm32_pfm3_priv =
{
  .ops                    = &stm32_pfm_ops,
  .tim                    = 0,
  .timer_id               = 11,
  .compare_channel        = 1,
  .state                  = STM32_PFM_STATE_IDLE,
  .shadow_ccr             = 0,
  .current_leading_period = 0,
  .cycle_count            = 0
};
#endif

#ifdef CONFIG_STM32F7_PFM4
struct stm32_pfm_priv_s stm32_pfm4_priv =
{
  .ops                    = &stm32_pfm_ops,
  .tim                    = 0,
  /* Channel 1 is unavailable */
  .timer_id               = 12,
  .compare_channel        = 2,
  .state                  = STM32_PFM_STATE_IDLE,
  .shadow_ccr             = 0,
  .current_leading_period = 0,
  .cycle_count            = 0
};
#endif

#ifdef CONFIG_STM32F7_PFM5
struct stm32_pfm_priv_s stm32_pfm5_priv =
{
  .ops                    = &stm32_pfm_ops,
  .tim                    = 0,
  .timer_id               = 13,
  .compare_channel        = 1,
  .state                  = STM32_PFM_STATE_IDLE,
  .shadow_ccr             = 0,
  .current_leading_period = 0,
  .cycle_count            = 0
};
#endif

#ifdef CONFIG_STM32F7_PFM6
struct stm32_pfm_priv_s stm32_pfm6_priv =
{
  .ops                    = &stm32_pfm_ops,
  .tim                    = 0,
  .timer_id               = 14,
  .compare_channel        = 1,
  .state                  = STM32_PFM_STATE_IDLE,
  .shadow_ccr             = 0,
  .current_leading_period = 0,
  .cycle_count            = 0
};
#endif

/************************************************************************************
 * Name: stm32_interrupt
 *
 * Description:
 *   Common timer interrupt handling.  NOTE: Only 16-bit timers require timer
 *   interrupts.
 *
 ************************************************************************************/

static int stm32_pfm_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct stm32_pfm_priv_s *priv = (FAR struct stm32_pfm_priv_s *)arg;
  uint32_t regval;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);

  /* Verify that this is an capture/compare interrupt.  Do noting if something
  else. */

  regval = priv->tim->ops->checkint(priv->tim, (1 << priv->compare_channel));
  if( regval == 0 )
    return OK;

  /*
   * The code is ugly and long but each interrupt requires only around
   * 8 - 10 C-instructions.
   */

  /* Disable the update/global interrupt at the NVIC */
  flags = enter_critical_section();

  priv->tim->ops->ackint(priv->tim, (1 << priv->compare_channel));
  switch( priv->state )
  {
    case STM32_PFM_STATE_LEADING:
      /* Go to SHOT state. The output is already high.
       *  Prepare for the end of SHOT state.
       */
      priv->shadow_ccr += CONFIG_STM32_PFM_SHOT_PEROID;
      priv->tim->ops->setcompare(priv->tim, priv->compare_channel, priv->shadow_ccr);
      priv->tim->ops->setchannel(priv->tim, priv->compare_channel, STM32_TIM_CH_OUTINACT);

      priv->state = STM32_PFM_STATE_SHOT;
      break;

    case STM32_PFM_STATE_SHOT:
      /* Reduce counter. If not finished yet, then re-enger the LEADING again
       * Alsom prepare for the end of LEADING state.
       */
      
      priv->cycle_count -= 1;
      if( priv->cycle_count == 0 )
      {
        _info("PFM FINISH\n");
        /* All pulses are done! Finished */
        priv->tim->ops->setchannel(priv->tim, priv->compare_channel, STM32_TIM_CH_OUTLO);
        priv->tim->ops->disableint(priv->tim, (1 << priv->compare_channel));
        priv->state = STM32_PFM_STATE_IDLE;
      }
      else
      {
        priv->shadow_ccr += priv->current_leading_period;
        priv->tim->ops->setcompare(priv->tim, priv->compare_channel, priv->shadow_ccr);
        priv->tim->ops->setchannel(priv->tim, priv->compare_channel, STM32_TIM_CH_OUTACT);
        priv->state = STM32_PFM_STATE_LEADING;
      }
      break;

    case STM32_PFM_STATE_IDLE:
      /* We should not be here but its ok */
      priv->tim->ops->setchannel(priv->tim, priv->compare_channel, STM32_TIM_CH_OUTLO);
      priv->tim->ops->disableint(priv->tim, (1 << priv->compare_channel));
      priv->state = STM32_PFM_STATE_IDLE;
      break;

    default:
      /* Fail proof state. All unknown states should be transit to IDLE state */
      _warn("PFM TIMER %d UNKNOWN STATE = %d\n", priv->timer_id, priv->state );
      priv->tim->ops->setchannel(priv->tim, priv->compare_channel, STM32_TIM_CH_OUTLO);
      priv->tim->ops->disableint(priv->tim, (1 << priv->compare_channel));
      priv->state = STM32_PFM_STATE_IDLE;
      break;
  }

  /* Finish all critical process */
  leave_critical_section(flags);

  return OK;
}

static bool stm32_pfm_is_idle(FAR struct stm32_pfm_dev_s *dev)
{
  FAR struct stm32_pfm_priv_s *pfm = (FAR struct stm32_pfm_priv_s *)(dev);

  DEBUGASSERT(pfm != NULL);

  if(pfm->state == STM32_PFM_STATE_IDLE)
    return true;
  else
    return false;
}

/*****************************************************************
 * Function: stm32_pfm_stop
 *
 * Abruptly stop PFM function and return to idle state
 *
 *****************************************************************/
static void stm32_pfm_stop(FAR struct stm32_pfm_dev_s *dev)
{
  irqstate_t flags;

  FAR struct stm32_pfm_priv_s *pfm = (FAR struct stm32_pfm_priv_s *)(dev);

  DEBUGASSERT(pfm != NULL);

  /* Disable the update/global interrupt at the NVIC */
  flags = enter_critical_section();

  if(pfm->state != STM32_PFM_STATE_IDLE)
  {
    pfm->tim->ops->setchannel(pfm->tim, pfm->compare_channel, STM32_TIM_CH_OUTLO);
    pfm->tim->ops->disableint(pfm->tim, (1 << pfm->compare_channel));
    pfm->state = STM32_PFM_STATE_IDLE;
  }

  /* Finish all critical process */
  leave_critical_section(flags);
}

/*****************************************************************
 * Function: stm32_pfm_start
 *
 * Start PFM function. If the device is already started, its will
 * operate with new period after finishing current cycle.
 *
 *****************************************************************/
static void stm32_pfm_start(FAR struct stm32_pfm_dev_s *dev, 
            uint16_t leading_period, uint16_t total_cycle)
{
  irqstate_t flags;

  FAR struct stm32_pfm_priv_s *pfm = (FAR struct stm32_pfm_priv_s *)(dev);

  DEBUGASSERT(pfm != NULL);

  /* Disable the update/global interrupt at the NVIC */
  flags = enter_critical_section();

  _info("Starting PFM\n");
  pfm->current_leading_period = leading_period;
  pfm->cycle_count = total_cycle;

  if( pfm->state == STM32_PFM_STATE_IDLE)
  {
    /* The PFM is in IDLE state. Thus, start it. */
    pfm->shadow_ccr = pfm->tim->ops->getcapture(pfm->tim, pfm->compare_channel) + leading_period;
    pfm->tim->ops->setcompare(pfm->tim, pfm->compare_channel, pfm->shadow_ccr);
    pfm->tim->ops->setchannel(pfm->tim, pfm->compare_channel, STM32_TIM_CH_OUTACT);
    pfm->tim->ops->enableint(pfm->tim, (1 << pfm->compare_channel));
    pfm->state = STM32_PFM_STATE_LEADING;
  }

  /* Finish all critical process */
  leave_critical_section(flags);
}

/************************************************************************************
 * Public Function - Initialization
 ************************************************************************************/

/*****************************************************************
 * Function: stm32_pfm_init
 *
 * Initialize PFM device
 *
 *****************************************************************/
FAR struct stm32_pfm_dev_s *stm32_pfm_init(unsigned motor_id)
{
  DEBUGASSERT(motor_id <= 5);
  _info("Init PFM\n");
  
  switch(motor_id)
  {
#ifdef CONFIG_STM32F7_PFM1
    case 0:
      stm32_pfm1_priv.tim = stm32_tim_init(stm32_pfm1_priv.timer_id);
      if(stm32_pfm1_priv.tim != NULL)
      {
        stm32_pfm1_priv.tim->ops->setmode(stm32_pfm1_priv.tim, STM32_TIM_MODE_UP);
        stm32_pfm1_priv.tim->ops->setclock(stm32_pfm1_priv.tim, CONFIG_STM32_PFM_FREQ);
        stm32_pfm1_priv.tim->ops->setperiod(stm32_pfm1_priv.tim, 0xFFFFFFFFUL);
        stm32_pfm1_priv.tim->ops->setisr(stm32_pfm1_priv.tim, stm32_pfm_interrupt, 
                    &stm32_pfm1_priv, (1 << stm32_pfm1_priv.compare_channel));
        stm32_pfm1_priv.tim->ops->setchannel(stm32_pfm1_priv.tim, stm32_pfm1_priv.compare_channel, STM32_TIM_CH_OUTLO);
        stm32_pfm1_priv.tim->ops->disableint(stm32_pfm1_priv.tim, (1 << stm32_pfm1_priv.compare_channel));

        return((FAR struct stm32_pfm_dev_s*)(&stm32_pfm1_priv));
      }
      else
      {
        _err("Unable to setup PFM1.\n");
        return NULL;
      }
#endif

#ifdef CONFIG_STM32F7_PFM2
    case 1:
      stm32_pfm2_priv.tim = stm32_tim_init(stm32_pfm2_priv.timer_id);
      if(stm32_pfm2_priv.tim != NULL)
      {
        stm32_pfm2_priv.tim->ops->setmode(stm32_pfm2_priv.tim, STM32_TIM_MODE_UP);
        stm32_pfm2_priv.tim->ops->setclock(stm32_pfm2_priv.tim, CONFIG_STM32_PFM_FREQ);
        stm32_pfm2_priv.tim->ops->setperiod(stm32_pfm2_priv.tim, 0xFFFFFFFFUL);
        stm32_pfm2_priv.tim->ops->setisr(stm32_pfm2_priv.tim, stm32_pfm_interrupt, 
                    &stm32_pfm2_priv, (1 << stm32_pfm2_priv.compare_channel));
        stm32_pfm2_priv.tim->ops->setchannel(stm32_pfm2_priv.tim, stm32_pfm2_priv.compare_channel, STM32_TIM_CH_OUTLO);
        stm32_pfm2_priv.tim->ops->disableint(stm32_pfm2_priv.tim, (1 << stm32_pfm2_priv.compare_channel));
        return((FAR struct stm32_pfm_dev_s*)(&stm32_pfm2_priv));
      }
      else
      {
        _err("Unable to setup PFM2.");
        return NULL;
      }
#endif

#ifdef CONFIG_STM32F7_PFM3
    case 2:
      stm32_pfm3_priv.tim = stm32_tim_init(stm32_pfm3_priv.timer_id);
      if(stm32_pfm3_priv.tim != NULL)
      {
        stm32_pfm3_priv.tim->ops->setmode(stm32_pfm3_priv.tim, STM32_TIM_MODE_UP);
        stm32_pfm3_priv.tim->ops->setclock(stm32_pfm3_priv.tim, CONFIG_STM32_PFM_FREQ);
        stm32_pfm3_priv.tim->ops->setperiod(stm32_pfm3_priv.tim, 0xFFFFFFFFUL);
        stm32_pfm3_priv.tim->ops->setisr(stm32_pfm3_priv.tim, stm32_pfm_interrupt, 
                    &stm32_pfm3_priv, (1 << stm32_pfm3_priv.compare_channel));
        stm32_pfm3_priv.tim->ops->setchannel(stm32_pfm3_priv.tim, stm32_pfm3_priv.compare_channel, STM32_TIM_CH_OUTLO);
        stm32_pfm3_priv.tim->ops->disableint(stm32_pfm3_priv.tim, (1 << stm32_pfm3_priv.compare_channel));
        return((FAR struct stm32_pfm_dev_s*)(&stm32_pfm3_priv));
      }
      else
      {
        _err("Unable to setup PFM3.");
        return NULL;
      }
#endif

#ifdef CONFIG_STM32F7_PFM4
    case 3:
      stm32_pfm4_priv.tim = stm32_tim_init(stm32_pfm4_priv.timer_id);
      if(stm32_pfm4_priv.tim != NULL)
      {
        stm32_pfm4_priv.tim->ops->setmode(stm32_pfm4_priv.tim, STM32_TIM_MODE_UP);
        stm32_pfm4_priv.tim->ops->setclock(stm32_pfm4_priv.tim, CONFIG_STM32_PFM_FREQ);
        stm32_pfm4_priv.tim->ops->setperiod(stm32_pfm4_priv.tim, 0xFFFFFFFFUL);
        stm32_pfm4_priv.tim->ops->setisr(stm32_pfm4_priv.tim, stm32_pfm_interrupt, 
                    &stm32_pfm4_priv, (1 << stm32_pfm4_priv.compare_channel));
        stm32_pfm4_priv.tim->ops->setchannel(stm32_pfm4_priv.tim, stm32_pfm4_priv.compare_channel, STM32_TIM_CH_OUTLO);
        stm32_pfm4_priv.tim->ops->disableint(stm32_pfm4_priv.tim, (1 << stm32_pfm4_priv.compare_channel));
        return((FAR struct stm32_pfm_dev_s*)(&stm32_pfm4_priv));
      }
      else
      {
        _err("Unable to setup PFM4.");
        return NULL;
      }
#endif

#ifdef CONFIG_STM32F7_PFM5
    case 4:
      stm32_pfm5_priv.tim = stm32_tim_init(stm32_pfm5_priv.timer_id);
      if(stm32_pfm5_priv.tim != NULL)
      {
        stm32_pfm5_priv.tim->ops->setmode(stm32_pfm5_priv.tim, STM32_TIM_MODE_UP);
        stm32_pfm5_priv.tim->ops->setclock(stm32_pfm5_priv.tim, CONFIG_STM32_PFM_FREQ);
        stm32_pfm5_priv.tim->ops->setperiod(stm32_pfm5_priv.tim, 0xFFFFFFFFUL);
        stm32_pfm5_priv.tim->ops->setisr(stm32_pfm5_priv.tim, stm32_pfm_interrupt, 
                    &stm32_pfm5_priv, (1 << stm32_pfm5_priv.compare_channel));
        stm32_pfm5_priv.tim->ops->setchannel(stm32_pfm5_priv.tim, stm32_pfm5_priv.compare_channel, STM32_TIM_CH_OUTLO);
        stm32_pfm5_priv.tim->ops->disableint(stm32_pfm5_priv.tim, (1 << stm32_pfm5_priv.compare_channel));
       return((FAR struct stm32_pfm_dev_s*)(&stm32_pfm5_priv));
      }
      else
      {
        _err("Unable to setup PFM5.");
        return NULL;
      }
#endif

#ifdef CONFIG_STM32F7_PFM6
    case 5:
      stm32_pfm6_priv.tim = stm32_tim_init(stm32_pfm6_priv.timer_id);
      if(stm32_pfm6_priv.tim != NULL)
      {
        stm32_pfm6_priv.tim->ops->setmode(stm32_pfm6_priv.tim, STM32_TIM_MODE_UP);
        stm32_pfm6_priv.tim->ops->setclock(stm32_pfm6_priv.tim, CONFIG_STM32_PFM_FREQ);
        stm32_pfm6_priv.tim->ops->setperiod(stm32_pfm6_priv.tim, 0xFFFFFFFFUL);
        stm32_pfm6_priv.tim->ops->setisr(stm32_pfm6_priv.tim, stm32_pfm_interrupt, 
                    &stm32_pfm6_priv, (1 << stm32_pfm6_priv.compare_channel));
        stm32_pfm6_priv.tim->ops->setchannel(stm32_pfm6_priv.tim, stm32_pfm6_priv.compare_channel, STM32_TIM_CH_OUTLO);
        stm32_pfm6_priv.tim->ops->disableint(stm32_pfm6_priv.tim, (1 << stm32_pfm6_priv.compare_channel));
        return((FAR struct stm32_pfm_dev_s*)(&stm32_pfm6_priv));
      }
      else
      {
        _err("Unable to setup PFM6.");
        return NULL;
      }
#endif

    default:
      _err("Unrecognized motor ID %d\n", motor_id);
      return NULL;
  }
  return NULL;  /* This line should not be executed */
}

#else

# error "No PFM available"

#endif /* defined(CONFIG_STM32F7_PFM1 || ... || PFM6) */
