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
#include <time.h>

#include <arch/board/board.h>
#include <nuttx/drivers/pwm.h>
#include "chip.h"
#include "up_internal.h"
#include "up_arch.h"
#include "stm32_pwm.h"

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

#if (!defined(CONFIG_STM32F7_TIM9_PWM)) || defined (CONFIG_STM32F7_TIM9_ADC) || \
    defined(CONFIG_STM32F7_TIM9_DAC) || defined(CONFIG_STM32F7_TIM9_QE) || \
    !defined(CONFIG_STM32F7_TIM9)
#  undef CONFIG_STM32F7_PFM1
#endif
#if (!defined(CONFIG_STM32F7_TIM10_PWM)) || defined (CONFIG_STM32F7_TIM10_ADC) || \
    defined(CONFIG_STM32F7_TIM10_DAC) || defined(CONFIG_STM32F7_TIM10_QE) || \
    !defined(CONFIG_STM32F7_TIM10)
#  undef CONFIG_STM32F7_PFM2
#endif
#if (!defined(CONFIG_STM32F7_TIM11_PWM)) || defined (CONFIG_STM32F7_TIM11_ADC) || \
    defined(CONFIG_STM32F7_TIM11_DAC) || defined(CONFIG_STM32F7_TIM11_QE) || \
    !defined(CONFIG_STM32F7_TIM11)
#  undef CONFIG_STM32F7_PFM3
#endif
#if (!defined(CONFIG_STM32F7_TIM12_PWM)) || defined (CONFIG_STM32F7_TIM12_ADC) || \
    defined(CONFIG_STM32F7_TIM12_DAC) || defined(CONFIG_STM32F7_TIM12_QE) || \
    !defined(CONFIG_STM32F7_TIM12)
#  undef CONFIG_STM32F7_PFM4
#endif
#if (!defined(CONFIG_STM32F7_TIM13_PWM)) || defined (CONFIG_STM32F7_TIM13_ADC) || \
    defined(CONFIG_STM32F7_TIM13_DAC) || defined(CONFIG_STM32F7_TIM13_QE) || \
    !defined(CONFIG_STM32F7_TIM13)
#  undef CONFIG_STM32F7_PFM5
#endif
#if (!defined(CONFIG_STM32F7_TIM14_PWM)) || defined (CONFIG_STM32F7_TIM14_ADC) || \
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
  STM32_PFM_STATE_RUNNING
}stm32_pfm_state_t;

/* PFM Device Structure */

struct stm32_pfm_priv_s
{
  struct stm32_pfm_ops_s  *ops;
  uint8_t                 id;     /* PFM id for logging */
  uint32_t                base;   /* Base address of the specified timer device */
  stm32_pfm_state_t       xState;
  FAR struct pwm_lowerhalf_s  *pxLowLevelPWM;

  /* Operation variables */
  volatile uint32_t       u32CycleCount;
  volatile uint32_t       u32ActualCount;
};

/* Prototypes */
static void stm32_pfm_start(FAR struct stm32_pfm_dev_s *dev, 
            uint32_t frequency, uint32_t total_cycle);
static uint32_t stm32_pfm_stop(FAR struct stm32_pfm_dev_s *dev);
static uint32_t stm32_pfm_get_count(FAR struct stm32_pfm_dev_s *dev);
static bool stm32_pfm_is_idle(FAR struct stm32_pfm_dev_s *dev);

/************************************************************************************
 * Private Functions
 ************************************************************************************/
/* Get a 16-bit register value by offset */

static inline uint16_t stm32_getreg16(FAR struct stm32_pfm_priv_s *dev,
                                      uint8_t offset)
{
  return getreg16(((struct stm32_pfm_priv_s *)dev)->base + offset);
}

/* Put a 16-bit register value by offset */

static inline void stm32_putreg16(FAR struct stm32_pfm_priv_s *dev, uint8_t offset,
                                  uint16_t value)
{
  putreg16(value, ((struct stm32_pfm_priv_s *)dev)->base + offset);
}

/* Modify a 16-bit register value by offset */

static inline void stm32_modifyreg16(FAR struct stm32_pfm_priv_s *dev,
                                     uint8_t offset, uint16_t clearbits,
                                     uint16_t setbits)
{
  modifyreg16(((struct stm32_pfm_priv_s *)dev)->base + offset, clearbits, setbits);
}

/************************************************************************************
 * Interrupt utilities.
 ************************************************************************************/
static int stm32_pfm_setisr(FAR struct stm32_pfm_priv_s *dev, xcpt_t handler)
{
  int vectorno;

  DEBUGASSERT(dev != NULL);

  switch (((struct stm32_pfm_priv_s *)dev)->base)
    {
#ifdef CONFIG_STM32F7_PFM1
      case STM32_TIM9_BASE:
        vectorno = STM32_IRQ_TIM9;
        break;
#endif
#ifdef CONFIG_STM32F7_PFM2
      case STM32_TIM10_BASE:
        vectorno = STM32_IRQ_TIM10;
        break;
#endif
#ifdef CONFIG_STM32F7_PFM3
      case STM32_TIM11_BASE:
        vectorno = STM32_IRQ_TIM11;
        break;
#endif
#ifdef CONFIG_STM32F7_PFM4
      case STM32_TIM12_BASE:
        vectorno = STM32_IRQ_TIM12;
        break;
#endif
#ifdef CONFIG_STM32F7_PFM5
      case STM32_TIM13_BASE:
        vectorno = STM32_IRQ_TIM13;
        break;
#endif
#ifdef CONFIG_STM32F7_PFM6
      case STM32_TIM14_BASE:
        vectorno = STM32_IRQ_TIM14;
        break;
#endif

      default:
        return -EINVAL;
    }

  /* Disable interrupt when callback is removed */

  if (!handler)
    {
      up_disable_irq(vectorno);
      irq_detach(vectorno);
      return OK;
    }

  /* Otherwise set callback and enable interrupt */

  irq_attach(vectorno, handler, (void*)dev);
  up_enable_irq(vectorno);

  return OK;
}

static void stm32_pfm_enableint(FAR struct stm32_pfm_priv_s *dev)
{
  DEBUGASSERT(dev != NULL);
  stm32_modifyreg16(dev, STM32_GTIM_DIER_OFFSET, 0, 1); /* UIE bit */
}

static void stm32_pfm_disableint(FAR struct stm32_pfm_priv_s *dev)
{
  DEBUGASSERT(dev != NULL);
  stm32_modifyreg16(dev, STM32_GTIM_DIER_OFFSET, 1, 0); /* UIE bit */
}

static int stm32_pfm_checkint(FAR struct stm32_pfm_priv_s *dev)
{
  uint16_t regval = stm32_getreg16(dev, STM32_GTIM_SR_OFFSET);
  return (regval & 1) ? 1 : 0;  /* UIF bit */
}

static void stm32_pfm_ackint(FAR struct stm32_pfm_priv_s *dev)
{
  stm32_putreg16(dev, STM32_GTIM_SR_OFFSET, (uint16_t)(~1U)); /* UIF bit */
}

/************************************************************************************
 * Name: stm32_interrupt
 *
 * Description: The central ISR for PFM devices.
 *   Common timer interrupt handling. The routing control PFM cycle counting.
 *
 ************************************************************************************/

static int stm32_pfm_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct stm32_pfm_priv_s *dev = (FAR struct stm32_pfm_priv_s *)arg;
  irqstate_t flags;

  DEBUGASSERT(dev != NULL);
  UNUSED(irq);
  UNUSED(context);

  /* Verify that this is an capture/compare interrupt.  Do noting if something
  else. */

  if( stm32_pfm_checkint( dev ) == 0 )
    return OK;

  stm32_pfm_ackint(dev);

  /* If current cycle count is zero, then the PFM completes its job */
  if( dev->u32CycleCount == 0 )
  {
    stm32_pfm_disableint( dev );
    dev->pxLowLevelPWM->ops->stop( dev->pxLowLevelPWM );
    dev->xState = STM32_PFM_STATE_IDLE;
    //_info("FInish PFM\n");
  }
  else
  {
    /* Otherwise, we continue with counter decrement */
    /* Disable the update/global interrupt at the NVIC */
    flags = enter_critical_section();
    (dev->u32CycleCount)--;
    (dev->u32ActualCount)++;
    /* Finish all critical process */
    leave_critical_section(flags);

  }

  return OK;
}


/************************************************************************************
 * Core Functions and Data
 ************************************************************************************/
struct stm32_pfm_ops_s stm32_pfm_ops =
{
  .start    = stm32_pfm_start,
  .stop     = stm32_pfm_stop,
  .get_count = stm32_pfm_get_count,
  .is_idle  = stm32_pfm_is_idle
};

#ifdef CONFIG_STM32F7_PFM1
struct stm32_pfm_priv_s stm32_pfm1_priv =
{
  .id             = 0,
  .ops            = &stm32_pfm_ops,
  .base           = STM32_TIM9_BASE,
  .xState         = STM32_PFM_STATE_IDLE,
  .pxLowLevelPWM  = 0,
  .u32ActualCount = 0,
  .u32CycleCount  = 0
};
#endif

#ifdef CONFIG_STM32F7_PFM2
struct stm32_pfm_priv_s stm32_pfm2_priv =
{
  .id             = 1,
  .ops            = &stm32_pfm_ops,
  .base           = STM32_TIM10_BASE,
  .xState         = STM32_PFM_STATE_IDLE,
  .pxLowLevelPWM  = 0,
  .u32ActualCount = 0,
  .u32CycleCount  = 0
};
#endif

#ifdef CONFIG_STM32F7_PFM3
struct stm32_pfm_priv_s stm32_pfm3_priv =
{
  .id             = 2,
  .ops            = &stm32_pfm_ops,
  .base           = STM32_TIM11_BASE,
  .xState         = STM32_PFM_STATE_IDLE,
  .pxLowLevelPWM  = 0,
  .u32ActualCount = 0,
  .u32CycleCount  = 0
};
#endif

#ifdef CONFIG_STM32F7_PFM4
struct stm32_pfm_priv_s stm32_pfm4_priv =
{
  .id             = 3,
  .ops            = &stm32_pfm_ops,
  .base           = STM32_TIM12_BASE,
  .xState         = STM32_PFM_STATE_IDLE,
  .pxLowLevelPWM  = 0,
  .u32ActualCount = 0,
  .u32CycleCount  = 0
};
#endif

#ifdef CONFIG_STM32F7_PFM5
struct stm32_pfm_priv_s stm32_pfm5_priv =
{
  .id             = 4,
  .ops            = &stm32_pfm_ops,
  .base           = STM32_TIM13_BASE,
  .xState         = STM32_PFM_STATE_IDLE,
  .pxLowLevelPWM  = 0,
  .u32ActualCount = 0,
  .u32CycleCount  = 0
};
#endif

#ifdef CONFIG_STM32F7_PFM6
struct stm32_pfm_priv_s stm32_pfm6_priv =
{
  .id             = 5,
  .ops            = &stm32_pfm_ops,
  .base           = STM32_TIM14_BASE,
  .xState         = STM32_PFM_STATE_IDLE,
  .pxLowLevelPWM  = 0,
  .u32ActualCount = 0,
  .u32CycleCount  = 0
};
#endif


static bool stm32_pfm_is_idle(FAR struct stm32_pfm_dev_s *dev)
{
  FAR struct stm32_pfm_priv_s *pfm = (FAR struct stm32_pfm_priv_s *)(dev);

  DEBUGASSERT(pfm != NULL);

  if(pfm->xState == STM32_PFM_STATE_RUNNING)
    return false;
  else
    return true;
}

/*****************************************************************
 * Function: stm32_pfm_stop
 *
 * Abruptly stop PFM function and return to idle state
 *
 *****************************************************************/
static uint32_t stm32_pfm_stop(FAR struct stm32_pfm_dev_s *dev)
{
  irqstate_t flags;

  FAR struct stm32_pfm_priv_s *pfm = (FAR struct stm32_pfm_priv_s *)(dev);

  DEBUGASSERT(pfm != NULL);

  stm32_pfm_disableint( pfm );
  pfm->pxLowLevelPWM->ops->stop( pfm->pxLowLevelPWM );

  /* Disable the update/global interrupt at the NVIC */
  flags = enter_critical_section();

  pfm->u32CycleCount = 0;

  /* Finish all critical process */
  leave_critical_section(flags);
  pfm->xState = STM32_PFM_STATE_IDLE;

  // _info( ":P;%lu;%u;stop=%lu\n", clock(), pfm->id, pfm->u32ActualCount );

  return( pfm->u32ActualCount );
}

/*****************************************************************
 * Function: stm32_pfm_get_count
 *
 * Return the current value of PFM cycle counter
 *
 *****************************************************************/
static uint32_t stm32_pfm_get_count(FAR struct stm32_pfm_dev_s *dev)
{
  FAR struct stm32_pfm_priv_s *pfm = (FAR struct stm32_pfm_priv_s *)(dev);
  //_info( ":P;%lu;%u;read=%lu\n", clock(), pfm->id, pfm->u32ActualCount );
  return( pfm->u32ActualCount );
}


/*****************************************************************
 * Function: stm32_pfm_start
 *
 * Start PFM function. If the device is already started, its will
 * operate with new period after finishing current cycle.
 *
 *****************************************************************/
#define CONFIG_STM32_PFM_SHOT_PEROID  10UL   /* pluse length in microsecond, should be 10 - 15 us */

static void stm32_pfm_start(FAR struct stm32_pfm_dev_s *dev, 
            uint32_t frequency, uint32_t total_cycle)
{
  irqstate_t flags;
  struct pwm_info_s pwm_info;

  FAR struct stm32_pfm_priv_s *pfm = (FAR struct stm32_pfm_priv_s *)(dev);

  DEBUGASSERT(pfm != NULL);

  //_info( ":P;%lu;%u;start;f=%lu;c=%lu\n", clock(), pfm->id, frequency, total_cycle );

  pwm_info.frequency = frequency;

  /* The PWM module operates in mode 2 which is active-low.
   * Therefore, off_period is the shot-period.
   * Duty cycle = off_period / T = off_period * frequency
   * The duty cycle is then convert into the ratio that 
   * 100% (1.0) duty-cycle = 65536. Hence, the final
   * settting value is frequency * off_period * 65530.
   * Also, please remark that the off_period is in unit second. (not microsecond)
   */
  pwm_info.duty = (uint16_t)((((uint64_t)(frequency * CONFIG_STM32_PFM_SHOT_PEROID )) * 65536UL ) / 1000000UL); /* Off-period is in micro second */
  //_info("PFM freq %u, duty %u, total %u\n", pwm_info.frequency, pwm_info.duty, total_cycle);

  pfm->xState = STM32_PFM_STATE_RUNNING;
  pfm->pxLowLevelPWM->ops->start(pfm->pxLowLevelPWM, &pwm_info);

  /* Disable the update/global interrupt at the NVIC */
  flags = enter_critical_section();

  pfm->u32CycleCount = total_cycle;
  pfm->u32ActualCount = 0UL;

  /* Finish all critical process */
  leave_critical_section(flags);

  stm32_pfm_enableint( pfm );
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
      stm32_pfm1_priv.pxLowLevelPWM = stm32_pwminitialize(9);
      if(stm32_pfm1_priv.pxLowLevelPWM != NULL)
      {
        stm32_pfm1_priv.pxLowLevelPWM->ops->setup(stm32_pfm1_priv.pxLowLevelPWM);

        stm32_pfm_setisr(&stm32_pfm1_priv, stm32_pfm_interrupt);
        stm32_pfm_disableint(&stm32_pfm1_priv);

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
      stm32_pfm2_priv.pxLowLevelPWM = stm32_pwminitialize(10);
      if(stm32_pfm2_priv.pxLowLevelPWM != NULL)
      {
        stm32_pfm2_priv.pxLowLevelPWM->ops->setup(stm32_pfm2_priv.pxLowLevelPWM);

        stm32_pfm_setisr(&stm32_pfm2_priv, stm32_pfm_interrupt);
        stm32_pfm_disableint(&stm32_pfm2_priv);

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
      stm32_pfm3_priv.pxLowLevelPWM = stm32_pwminitialize(11);
      if(stm32_pfm3_priv.pxLowLevelPWM != NULL)
      {
        stm32_pfm3_priv.pxLowLevelPWM->ops->setup(stm32_pfm3_priv.pxLowLevelPWM);

        stm32_pfm_setisr(&stm32_pfm3_priv, stm32_pfm_interrupt);
        stm32_pfm_disableint(&stm32_pfm3_priv);

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
      stm32_pfm4_priv.pxLowLevelPWM = stm32_pwminitialize(12);
      if(stm32_pfm4_priv.pxLowLevelPWM != NULL)
      {
        stm32_pfm4_priv.pxLowLevelPWM->ops->setup(stm32_pfm4_priv.pxLowLevelPWM);

        stm32_pfm_setisr(&stm32_pfm4_priv, stm32_pfm_interrupt);
        stm32_pfm_disableint(&stm32_pfm4_priv);

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
      stm32_pfm5_priv.pxLowLevelPWM = stm32_pwminitialize(13);
      if(stm32_pfm5_priv.pxLowLevelPWM != NULL)
      {
        stm32_pfm5_priv.pxLowLevelPWM->ops->setup(stm32_pfm5_priv.pxLowLevelPWM);

        stm32_pfm_setisr(&stm32_pfm5_priv, stm32_pfm_interrupt);
        stm32_pfm_disableint(&stm32_pfm5_priv);

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
      stm32_pfm6_priv.pxLowLevelPWM = stm32_pwminitialize(14);
      if(stm32_pfm6_priv.pxLowLevelPWM != NULL)
      {
        stm32_pfm6_priv.pxLowLevelPWM->ops->setup(stm32_pfm6_priv.pxLowLevelPWM);

        stm32_pfm_setisr(&stm32_pfm6_priv, stm32_pfm_interrupt);
        stm32_pfm_disableint(&stm32_pfm6_priv);

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
