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
#include "hardware/stm32_tim.h"

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
    defined(CONFIG_STM32F7_TIM9)
#  undef CONFIG_STM32F7_PFM1
#endif
#if defined(CONFIG_STM32F7_TIM10_PWM) || defined (CONFIG_STM32F7_TIM10_ADC) || \
    defined(CONFIG_STM32F7_TIM10_DAC) || defined(CONFIG_STM32F7_TIM10_QE) || \
    defined(CONFIG_STM32F7_TIM10)
#  undef CONFIG_STM32F7_PFM2
#endif
#if defined(CONFIG_STM32F7_TIM11_PWM) || defined (CONFIG_STM32F7_TIM11_ADC) || \
    defined(CONFIG_STM32F7_TIM11_DAC) || defined(CONFIG_STM32F7_TIM11_QE) || \
    defined(CONFIG_STM32F7_TIM11)
#  undef CONFIG_STM32F7_PFM3
#endif
#if defined(CONFIG_STM32F7_TIM12_PWM) || defined (CONFIG_STM32F7_TIM12_ADC) || \
    defined(CONFIG_STM32F7_TIM12_DAC) || defined(CONFIG_STM32F7_TIM12_QE) || \
    defined(CONFIG_STM32F7_TIM12)
#  undef CONFIG_STM32F7_PFM4
#endif
#if defined(CONFIG_STM32F7_TIM13_PWM) || defined (CONFIG_STM32F7_TIM13_ADC) || \
    defined(CONFIG_STM32F7_TIM13_DAC) || defined(CONFIG_STM32F7_TIM13_QE) || \
    defined(CONFIG_STM32F7_TIM13)
#  undef CONFIG_STM32F7_PFM5
#endif
#if defined(CONFIG_STM32F7_TIM14_PWM) || defined (CONFIG_STM32F7_TIM14_ADC) || \
    defined(CONFIG_STM32F7_TIM14_DAC) || defined(CONFIG_STM32F7_TIM14_QE) || \
    defined(CONFIG_STM32F7_TIM14)
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

/* Command used with IOCTL */
typedef enum
{
  PFMIOC_STOP,
  PFMIOC_SOFT_START,
  PFMIOC_PREEMP_START,
}stm32_pfm_ioc_cmd_t;

typedef struct
{
  uint16_t  leading_period;
  uint16_t  cycle_count;
}stm32_pfm_param_t;

/* PFM Device Structure */

struct stm32_pfm_priv_s
{
  FAR struct stm32_pfm_ops_s  *ops;
  uint32_t                    base;   /* TIMn base address */
  uint32_t                    oc_mask;  /* Mask for OC control bits */
  uint8_t                     oc_shift; /* Number of bit to shift for OC control */
  uint8_t                     ccr_offset; /* Offset to CCR register */
  uint16_t                    cc_en;  /* Capture/Compare Enable */
  uint16_t                    dier_bit_mask; /* Mask for channel interrupt */
  uint16_t                    cc_int; /* Capture/Compare interrupt bit */
  int                         vectorno; /* Global interrupt vector number */

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
 * Low-level Functions
 ************************************************************************************/

/* Get a 16-bit register value by offset */

static inline uint16_t stm32_getreg16(FAR struct stm32_pfm_priv_s *pfm,
                                      uint8_t offset)
{
  return getreg16(pfm->base + offset);
}

/* Put a 16-bit register value by offset */

static inline void stm32_putreg16(FAR struct stm32_pfm_priv_s *pfm, uint8_t offset,
                                  uint16_t value)
{
  putreg16(value, pfm->base + offset);
}

/* Modify a 16-bit register value by offset */

static inline void stm32_modifyreg16(FAR struct stm32_pfm_priv_s *pfm,
                                     uint8_t offset, uint16_t clearbits,
                                     uint16_t setbits)
{
  modifyreg16(pfm->base + offset, clearbits, setbits);
}

/* Get a 32-bit register value by offset.  This applies only for the STM32 F4
 * 32-bit registers (CNT, ARR, CRR1-4) in the 32-bit timers TIM2-5.
 */

static inline uint32_t stm32_getreg32(FAR struct stm32_pfm_priv_s *pfm,
                                      uint8_t offset)
{
  return getreg32(pfm->base + offset);
}

/* Put a 32-bit register value by offset.  This applies only for the STM32 F4
 * 32-bit registers (CNT, ARR, CRR1-4) in the 32-bit timers TIM2-5.
 */

static inline void stm32_putreg32(FAR struct stm32_pfm_priv_s *pfm, uint8_t offset,
                                  uint32_t value)
{
  putreg32(value, pfm->base + offset);
}

static inline void stm32_enable_globalint(int vectorno)
{
  up_enable_irq(vectorno);
}

static inline void stm32_disable_globalint(int vectorno)
{
  up_disable_irq(vectorno);
}

/************************************************************************************
 * Basic Functions
 ************************************************************************************/

static void stm32_pfm_enableint(FAR struct stm32_pfm_priv_s *pfm)
{
  DEBUGASSERT(pfm != NULL);
  stm32_modifyreg16(pfm, STM32_GTIM_DIER_OFFSET, 0, pfm->dier_bit_mask);
}

static void stm32_pfm_disableint(FAR struct stm32_pfm_priv_s *pfm)
{
  DEBUGASSERT(pfm != NULL);
  stm32_modifyreg16(pfm, STM32_GTIM_DIER_OFFSET, pfm->dier_bit_mask, 0);
}

static void stm32_pfm_reload_counter(FAR struct stm32_pfm_priv_s *pfm)
{
  DEBUGASSERT(pfm != NULL);
  uint16_t val = stm32_getreg16(pfm, STM32_GTIM_EGR_OFFSET);
  val |= GTIM_EGR_UG;
  stm32_putreg16(pfm, STM32_GTIM_EGR_OFFSET, val);
}

static void stm32_pfm_enable(FAR struct stm32_pfm_priv_s *pfm)
{
  DEBUGASSERT(pfm != NULL);
  uint16_t val = stm32_getreg16(pfm, STM32_GTIM_CR1_OFFSET);
  val |= GTIM_CR1_CEN;
  stm32_pfm_reload_counter(pfm);
  stm32_putreg16(pfm, STM32_GTIM_CR1_OFFSET, val);
}

static void stm32_pfm_disable(FAR struct stm32_pfm_priv_s *pfm)
{
  DEBUGASSERT(pfm != NULL);
  uint16_t val = stm32_getreg16(pfm, STM32_GTIM_CR1_OFFSET);
  val &= ~GTIM_CR1_CEN;
  stm32_putreg16(pfm, STM32_GTIM_CR1_OFFSET, val);
}

static void stm32_pfm_setcomparemode(FAR struct stm32_pfm_priv_s *pfm, 
                                uint32_t mode)
{
  DEBUGASSERT(pfm != NULL);

  uint32_t val = stm32_getreg32(pfm, STM32_GTIM_CCMR1_OFFSET);
  mode &= 7;  // Mask the mode to only usable bits
  val &= ~(pfm->oc_mask);
  val |= (mode << (pfm->oc_shift));
  stm32_putreg32(pfm, STM32_GTIM_CCMR1_OFFSET, val);
}

static void stm32_pfm_setperiod(FAR struct stm32_pfm_priv_s *pfm,
                                uint16_t period)
{
  DEBUGASSERT(pfm != NULL);
  stm32_putreg16(pfm, STM32_GTIM_ARR_OFFSET, period);
}

static int stm32_pfm_setclock(FAR struct stm32_pfm_priv_s *pfm, uint32_t freq)
{
  uint32_t freqin;
  int prescaler;

  DEBUGASSERT(pfm != NULL);

  /* Disable Timer? */

  if (freq == 0)
    {
      stm32_pfm_disable(pfm);
      return 0;
    }

  /* Get the input clock frequency for this timer.  These vary with
   * different timer clock sources, MCU-specific timer configuration, and
   * board-specific clock configuration.  The correct input clock frequency
   * must be defined in the board.h header file.
   */

  switch (pfm->base)
    {
#ifdef CONFIG_STM32F7_PFM1
      case STM32_TIM9_BASE:
        freqin = STM32_APB2_TIM9_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F7_PFM2
      case STM32_TIM10_BASE:
        freqin = STM32_APB2_TIM10_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F7_PFM3
      case STM32_TIM11_BASE:
        freqin = STM32_APB2_TIM11_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F7_PFM4
      case STM32_TIM12_BASE:
        freqin = STM32_APB1_TIM12_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F7_PFM5
      case STM32_TIM13_BASE:
        freqin = STM32_APB1_TIM13_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F7_PFM6
      case STM32_TIM14_BASE:
        freqin = STM32_APB1_TIM14_CLKIN;
        break;
#endif
      default:
        _err("..Unknown timer!!..");
        return -EINVAL;
    }

  /* Select a pre-scaler value for this timer using the input clock
   * frequency.
   */

  prescaler = freqin / freq;

  /* We need to decrement value for '1', but only, if that will not to
   * cause underflow.
   */

  if (prescaler > 0)
    {
      prescaler--;
    }

  /* Check for overflow as well. */

  if (prescaler > 0xffff)
    {
      prescaler = 0xffff;
    }

  stm32_putreg16(pfm, STM32_GTIM_PSC_OFFSET, prescaler);
  stm32_pfm_setperiod(pfm, 0xFFFF);
  stm32_pfm_enable(pfm);

  return prescaler;
}

static void stm32_pfm_setchannel(FAR struct stm32_pfm_priv_s *pfm)
{
  DEBUGASSERT(pfm != NULL);
  stm32_modifyreg16(pfm, STM32_GTIM_CCER_OFFSET, 0 ,(pfm->cc_en));
}

static int stm32_pfm_setuppfm(FAR struct stm32_pfm_priv_s *pfm)
{
  int retval;

  DEBUGASSERT(pfm != NULL);

  stm32_disable_globalint(pfm->vectorno);
  stm32_pfm_disableint(pfm);
  stm32_pfm_setcomparemode(pfm, ATIM_CCMR_MODE_OCREFLO);
  retval = stm32_pfm_setclock(pfm, CONFIG_STM32_PFM_FREQ);
  if( retval <= 0 )
  {
    return -EINVAL;
  }
  stm32_pfm_setchannel(pfm);
  return OK;
}


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
  .base                   = STM32_TIM9_BASE,
  .oc_mask                = GTIM_CCMR1_OC1M_MASK,
  .oc_shift               = GTIM_CCMR1_OC1M_SHIFT,
  .ccr_offset             = STM32_GTIM_CCR1_OFFSET,
  .cc_en                  = GTIM_CCER_CC1E,
  .cc_int                 = GTIM_SR_CC1IF,
  .dier_bit_mask          = GTIM_DIER_CC1IE,
  .vectorno               = STM32_IRQ_TIM9,
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
  .base                   = STM32_TIM10_BASE,
  .oc_mask                = GTIM_CCMR1_OC1M_MASK, 
  .oc_shift               = GTIM_CCMR1_OC1M_SHIFT,
  .ccr_offset             = STM32_GTIM_CCR1_OFFSET,
  .cc_en                  = GTIM_CCER_CC1E,
  .cc_int                 = GTIM_SR_CC1IF,
  .dier_bit_mask          = GTIM_DIER_CC1IE,
  .vectorno               = STM32_IRQ_TIM10,
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
  .base                   = STM32_TIM11_BASE,
  .oc_mask                = GTIM_CCMR1_OC1M_MASK,
  .oc_shift               = GTIM_CCMR1_OC1M_SHIFT,
  .ccr_offset             = STM32_GTIM_CCR1_OFFSET,
  .cc_en                  = GTIM_CCER_CC1E,
  .cc_int                 = GTIM_SR_CC1IF,
  .dier_bit_mask          = GTIM_DIER_CC1IE,
  .vectorno               = STM32_IRQ_TIM11,
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
  .base                   = STM32_TIM12_BASE,
  /* Channel 1 is unavailable */
  .oc_mask                = GTIM_CCMR1_OC2M_MASK,
  .oc_shift               = GTIM_CCMR1_OC2M_SHIFT,
  .ccr_offset             = STM32_GTIM_CCR2_OFFSET,
  .cc_en                  = GTIM_CCER_CC2E,
  .cc_int                 = GTIM_SR_CC2IF,
  .dier_bit_mask          = GTIM_DIER_CC2IE,
  .vectorno               = STM32_IRQ_TIM12,
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
  .base                   = STM32_TIM13_BASE,
  .oc_mask                = GTIM_CCMR1_OC1M_MASK,
  .oc_shift               = GTIM_CCMR1_OC1M_SHIFT,
  .ccr_offset             = STM32_GTIM_CCR1_OFFSET,
  .cc_en                  = GTIM_CCER_CC1E,
  .cc_int                 = GTIM_SR_CC1IF,
  .dier_bit_mask          = GTIM_DIER_CC1IE,
  .vectorno               = STM32_IRQ_TIM13,
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
  .base                   = STM32_TIM14_BASE,
  .oc_mask                = GTIM_CCMR1_OC1M_MASK,
  .oc_shift               = GTIM_CCMR1_OC1M_SHIFT,
  .ccr_offset             = STM32_GTIM_CCR1_OFFSET,
  .cc_en                  = GTIM_CCER_CC1E,
  .cc_int                 = GTIM_SR_CC1IF,
  .dier_bit_mask          = GTIM_DIER_CC1IE,
  .vectorno               = STM32_IRQ_TIM14,
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

  /* Verify that this is an capture/compare interrupt.  Nothing else is expected. */

  regval = stm32_getreg16(priv, STM32_GTIM_SR_OFFSET);
  DEBUGASSERT(((regval & GTIM_SR_CC1IF) | (regval & GTIM_SR_CC2IF)) != 0);

  /*
   * The code is ugly and long but each interrupt requires only around
   * 8 - 10 C-instructions.
   */

  /* Disable the update/global interrupt at the NVIC */
  flags = enter_critical_section();

  stm32_putreg16(priv, STM32_GTIM_SR_OFFSET, ~(priv->cc_int));
  switch( priv->state )
  {
    case STM32_PFM_STATE_LEADING:
      /* Go to SHOT state. The output is already high.
       *  Prepare for the end of SHOT state.
       */
      priv->shadow_ccr += CONFIG_STM32_PFM_SHOT_PEROID;
      stm32_putreg16(priv, priv->ccr_offset, priv->shadow_ccr);

      regval = stm32_getreg32(priv, STM32_GTIM_CCMR1_OFFSET);
      regval &= ~(priv->oc_mask);
      regval |= (GTIM_CCMR_MODE_CHINACT << (priv->oc_shift));
      stm32_putreg32(priv, STM32_GTIM_CCMR1_OFFSET, regval);
      priv->state = STM32_PFM_STATE_SHOT;
      break;

    case STM32_PFM_STATE_SHOT:
      /* Reduce counter. If not finished yet, then re-enger the LEADING again
       * Alsom prepare for the end of LEADING state.
       */
      priv->cycle_count -= 1;
      if( priv->cycle_count == 0 )
      {
        /* All pulses are done! Finished */
        regval = stm32_getreg32(priv, STM32_GTIM_CCMR1_OFFSET);
        regval &= ~(priv->oc_mask);
        regval |= (GTIM_CCMR_MODE_OCREFLO << (priv->oc_shift));
        stm32_putreg32(priv, STM32_GTIM_CCMR1_OFFSET, regval);
        stm32_modifyreg16(priv, STM32_GTIM_DIER_OFFSET, (priv->dier_bit_mask), 0);
        priv->state = STM32_PFM_STATE_IDLE;
      }
      else
      {
        priv->shadow_ccr += CONFIG_STM32_PFM_SHOT_PEROID;
        stm32_putreg16(priv, priv->ccr_offset, priv->shadow_ccr);

        regval = stm32_getreg32(priv, STM32_GTIM_CCMR1_OFFSET);
        regval &= ~(priv->oc_mask);
        regval |= (GTIM_CCMR_MODE_CHINACT << (priv->oc_shift));
        stm32_putreg32(priv, STM32_GTIM_CCMR1_OFFSET, regval);
        priv->state = STM32_PFM_STATE_LEADING;
      }
      break;

    default:
      /* Fail proof state. All unknown states should be transit to IDLE state */
      regval = stm32_getreg32(priv, STM32_GTIM_CCMR1_OFFSET);
      regval &= ~(priv->oc_mask);
      regval |= (GTIM_CCMR_MODE_OCREFLO << (priv->oc_shift));
      stm32_putreg32(priv, STM32_GTIM_CCMR1_OFFSET, regval);
      stm32_modifyreg16(priv, STM32_GTIM_DIER_OFFSET, (priv->dier_bit_mask), 0);
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
  uint32_t regval;
  irqstate_t flags;

  FAR struct stm32_pfm_priv_s *pfm = (FAR struct stm32_pfm_priv_s *)(dev);

  DEBUGASSERT(pfm != NULL);

  /* Disable the update/global interrupt at the NVIC */
  flags = enter_critical_section();

  if(pfm->state != STM32_PFM_STATE_IDLE)
  {
    regval = stm32_getreg32(pfm, STM32_GTIM_CCMR1_OFFSET);
    regval &= ~(pfm->oc_mask);
    regval |= (GTIM_CCMR_MODE_OCREFLO << (pfm->oc_shift));
    stm32_putreg32(pfm, STM32_GTIM_CCMR1_OFFSET, regval);
    stm32_pfm_disable(pfm);
    stm32_pfm_disableint(pfm);
    stm32_disable_globalint(pfm->vectorno);
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
  uint32_t regval;

  FAR struct stm32_pfm_priv_s *pfm = (FAR struct stm32_pfm_priv_s *)(dev);

  DEBUGASSERT(pfm != NULL);

  /* Disable the update/global interrupt at the NVIC */
  flags = enter_critical_section();

  pfm->current_leading_period = leading_period;
  pfm->cycle_count = total_cycle;

  if( pfm->state == STM32_PFM_STATE_IDLE)
  {
    /* The PFM is in IDLE state. Thus, start it. */
    pfm->shadow_ccr = stm32_getreg16(pfm, (pfm->ccr_offset)) + leading_period;
    stm32_putreg16(pfm, pfm->ccr_offset, pfm->shadow_ccr);

    regval = stm32_getreg32(pfm, STM32_GTIM_CCMR1_OFFSET);
    regval &= ~(pfm->oc_mask);
    regval |= (GTIM_CCMR_MODE_CHACT << (pfm->oc_shift));
    stm32_putreg32(pfm, STM32_GTIM_CCMR1_OFFSET, regval);
    pfm->state = STM32_PFM_STATE_LEADING;
    stm32_pfm_enableint(pfm);
    stm32_enable_globalint(pfm->vectorno);
    stm32_pfm_enable(pfm);
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
  int status;

  DEBUGASSERT(motor_id <= 5);
  
  switch(motor_id)
  {
#ifdef CONFIG_STM32F7_PFM1
    case 0:
      status = stm32_pfm_setuppfm(&stm32_pfm1_priv);
      if(status == OK)
      {
        stm32_configgpio(GPIO_TIM9_OUT);
        irq_attach(STM32_IRQ_TIM9, stm32_pfm_interrupt, (FAR void*)(&stm32_pfm1_priv));
        modifyreg32(STM32_RCC_APB2ENR, 0, RCC_APB2ENR_TIM9EN);
        return((FAR struct stm32_pfm_dev_s*)(&stm32_pfm1_priv));
      }
      else
      {
        _err("Unable to setup PFM1.");
        return NULL;
      }
#endif

#ifdef CONFIG_STM32F7_PFM2
    case 1:
      status = stm32_pfm_setuppfm(&stm32_pfm2_priv);
      if(status == OK)
      {
        stm32_configgpio(GPIO_TIM10_OUT);
        irq_attach(STM32_IRQ_TIM10, stm32_pfm_interrupt, (FAR void*)(&stm32_pfm2_priv));
        modifyreg32(STM32_RCC_APB2ENR, 0, RCC_APB2ENR_TIM10EN);
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
      status = stm32_pfm_setuppfm(&stm32_pfm3_priv);
      if(status == OK)
      {
        stm32_configgpio(GPIO_TIM11_OUT);
        irq_attach(STM32_IRQ_TIM11, stm32_pfm_interrupt, (FAR void*)(&stm32_pfm3_priv));
        modifyreg32(STM32_RCC_APB2ENR, 0, RCC_APB2ENR_TIM11EN);
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
      status = stm32_pfm_setuppfm(&stm32_pfm4_priv);
      if(status == OK)
      {
        stm32_configgpio(GPIO_TIM12_OUT);
        irq_attach(STM32_IRQ_TIM12, stm32_pfm_interrupt, (FAR void*)(&stm32_pfm4_priv));
        modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM12EN);
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
      status = stm32_pfm_setuppfm(&stm32_pfm5_priv);
      if(status == OK)
      {
        stm32_configgpio(GPIO_TIM13_OUT);
        irq_attach(STM32_IRQ_TIM13, stm32_pfm_interrupt, (FAR void*)(&stm32_pfm5_priv));
        modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM13EN);
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
      status = stm32_pfm_setuppfm(&stm32_pfm6_priv);
      if(status == OK)
      {
        stm32_configgpio(GPIO_TIM14_OUT);
        irq_attach(STM32_IRQ_TIM14, stm32_pfm_interrupt, (FAR void*)(&stm32_pfm6_priv));
        modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM14EN);
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
