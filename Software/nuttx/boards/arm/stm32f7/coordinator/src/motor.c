/************************************************************************************
 * configs/coordinator/motor.c
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
#include <nuttx/fs/fs.h>

#include <sys/types.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_internal.h"
#include "up_arch.h"
#include "stm32_gpio.h"
#include "stm32_pfm.h"
#include <arch/board/motor.h>

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* Motor configuration structure */
struct motor_cfg_s
{
  uint32_t  motor_id;     /* PFM id of this motor */
  uint32_t  dir_pin;      /* Motor direction pin */
  uint32_t  srv_on_pin;   /* Motor enable pin */
  uint32_t  alrm_res_pin; /* Alarm reset pin */
  uint32_t  dev_clr_pin;  /* Counter-deviation clear pin */
  uint32_t  alrm_pin;     /* Motor alarm input pin */
  uint32_t  cmplt_pin;    /* Command complete input pin */
  uint32_t  led_red;      /* Red LED control pin */
  uint32_t  led_green;    /* Green LED control pin */
};

struct motor_priv_s
{
  FAR struct motor_cfg_s      *cfg;
  FAR struct stm32_pfm_dev_s  *pfm;
  sem_t                       exclsem;  /* Supports mutual exclusion */
  uint32_t                    crefs;    /* File opening counter */
  volatile motor_state_t      state;
};

/************************************************************************************
 * Private Data
 ************************************************************************************/

struct motor_cfg_s motor_cfg[6] = 
{

{
  .motor_id     = 0,
  .dir_pin      = GPIO_MTR1_DIRECTION,
  .srv_on_pin   = GPIO_MTR1_SVON,
  .alrm_res_pin = GPIO_MTR1_ALRM_RES,
  .dev_clr_pin  = GPIO_MTR1_DEV_CLR,
  .alrm_pin     = GPIO_MTR1_ALRM,
  .cmplt_pin    = GPIO_MTR1_CMPLT,
  .led_red      = GPIO_MTR1_LED_RED,
  .led_green    = GPIO_MTR1_LED_GRN
},

{
  .motor_id     = 1,
  .dir_pin      = GPIO_MTR2_DIRECTION,
  .srv_on_pin   = GPIO_MTR2_SVON,
  .alrm_res_pin = GPIO_MTR2_ALRM_RES,
  .dev_clr_pin  = GPIO_MTR2_DEV_CLR,
  .alrm_pin     = GPIO_MTR2_ALRM,
  .cmplt_pin    = GPIO_MTR2_CMPLT,
  .led_red      = GPIO_MTR2_LED_RED,
  .led_green    = GPIO_MTR2_LED_GRN
},

{
  .motor_id     = 2,
  .dir_pin      = GPIO_MTR3_DIRECTION,
  .srv_on_pin   = GPIO_MTR3_SVON,
  .alrm_res_pin = GPIO_MTR3_ALRM_RES,
  .dev_clr_pin  = GPIO_MTR3_DEV_CLR,
  .alrm_pin     = GPIO_MTR3_ALRM,
  .cmplt_pin    = GPIO_MTR3_CMPLT,
  .led_red      = GPIO_MTR3_LED_RED,
  .led_green    = GPIO_MTR3_LED_GRN
},

{
  .motor_id     = 3,
  .dir_pin      = GPIO_MTR4_DIRECTION,
  .srv_on_pin   = GPIO_MTR4_SVON,
  .alrm_res_pin = GPIO_MTR4_ALRM_RES,
  .dev_clr_pin  = GPIO_MTR4_DEV_CLR,
  .alrm_pin     = GPIO_MTR4_ALRM,
  .cmplt_pin    = GPIO_MTR4_CMPLT,
  .led_red      = GPIO_MTR4_LED_RED,
  .led_green    = GPIO_MTR4_LED_GRN
},

{
  .motor_id     = 4,
  .dir_pin      = GPIO_MTR5_DIRECTION,
  .srv_on_pin   = GPIO_MTR5_SVON,
  .alrm_res_pin = GPIO_MTR5_ALRM_RES,
  .dev_clr_pin  = GPIO_MTR5_DEV_CLR,
  .alrm_pin     = GPIO_MTR5_ALRM,
  .cmplt_pin    = GPIO_MTR5_CMPLT,
  .led_red      = GPIO_MTR5_LED_RED,
  .led_green    = GPIO_MTR5_LED_GRN
},

{
  .motor_id     = 5,
  .dir_pin      = GPIO_MTR6_DIRECTION,
  .srv_on_pin   = GPIO_MTR6_SVON,
  .alrm_res_pin = GPIO_MTR6_ALRM_RES,
  .dev_clr_pin  = GPIO_MTR6_DEV_CLR,
  .alrm_pin     = GPIO_MTR6_ALRM,
  .cmplt_pin    = GPIO_MTR6_CMPLT,
  .led_red      = GPIO_MTR6_LED_RED,
  .led_green    = GPIO_MTR6_LED_GRN
}

};

struct motor_priv_s motor_priv[6] =
{

{
  .cfg    = &(motor_cfg[0]),
  .pfm    = NULL,
  .crefs  = 0,
  .state  = MOTOR_UNINIT
},

{
  .cfg    = &(motor_cfg[1]),
  .pfm    = NULL,
  .crefs  = 0,
  .state  = MOTOR_UNINIT
},

{
  .cfg    = &(motor_cfg[2]),
  .pfm    = NULL,
  .crefs  = 0,
  .state  = MOTOR_UNINIT
},

{
  .cfg    = &(motor_cfg[3]),
  .pfm    = NULL,
  .crefs  = 0,
  .state  = MOTOR_UNINIT
},

{
  .cfg    = &(motor_cfg[4]),
  .pfm    = NULL,
  .crefs  = 0,
  .state  = MOTOR_UNINIT
},

{
  .cfg    = &(motor_cfg[5]),
  .pfm    = NULL,
  .crefs  = 0,
  .state  = MOTOR_UNINIT
}

};

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Helper macro
 ************************************************************************************/
#define motor_setpin(pin)           stm32_gpiowrite(pin,true)
#define motor_respin(pin)           stm32_gpiowrite(pin,false)

/*#define motor_emergency_stat()      stm32_gpioread(GPIO_EMERGENGY_STOP)*/
#define motor_emergency_stat()      (false)
#define motor_alarm_stat(cfg)       stm32_gpioread(cfg->alrm_pin)
#define motor_complete_stat(cfg)    stm32_gpioread(cfg->cmplt_pin)

#define motor_set_cw(cfg)           motor_setpin(cfg->dir_pin)
#define motor_set_ccw(cfg)          motor_respin(cfg->dir_pin)

#define motor_servo_on(cfg)         motor_setpin(cfg->srv_on_pin)
#define motor_servo_off(cfg)        motor_respin(cfg->srv_on_pin)

#define motor_led_red_on(cfg)       motor_setpin(cfg->led_red)
#define motor_led_red_off(cfg)      motor_respin(cfg->led_red)
#define motor_led_green_on(cfg)     motor_setpin(cfg->led_green)
#define motor_led_green_off(cfg)    motor_respin(cfg->led_green)

#define motor_stop_pfm(motor)       (motor->pfm->ops->stop(motor->pfm))
#define motor_start_pfm(motor,p,c)  (motor->pfm->ops->start(motor->pfm,p,c))

/************************************************************************************
 * Low-level Functions
 ************************************************************************************/

static inline void motor_alarm_res(FAR struct motor_cfg_s* cfg)
{
  /* Sending a pulse to reset alarm status */
  motor_setpin(cfg->alrm_res_pin);
  for(int i = 100;i > 0;i--);    /* Delay for a moment */
  motor_respin(cfg->alrm_res_pin);
}

static inline void motor_clr_devcount(FAR struct motor_cfg_s* cfg)
{
  /* Sending a pulse to clear deviation counter */
  motor_setpin(cfg->dev_clr_pin);
  for(int i = 100;i > 0;i--);    /* Delay for a moment */
  motor_respin(cfg->dev_clr_pin);
}

static int motor_init_gpio(FAR struct motor_cfg_s* cfg)
{
  int retval;

  /* Config GPIO pins */
  retval = stm32_configgpio(cfg->dir_pin);
  if(retval != OK)
  {
    return retval;
  }
  retval = stm32_configgpio(cfg->srv_on_pin);
  if(retval != OK)
  {
    return retval;
  }
  retval = stm32_configgpio(cfg->alrm_res_pin);
  if(retval != OK)
  {
    return retval;
  }
  retval = stm32_configgpio(cfg->dev_clr_pin);
  if(retval != OK)
  {
    return retval;
  }
  retval = stm32_configgpio(cfg->alrm_pin);
  if(retval != OK)
  {
    return retval;
  }
  retval = stm32_configgpio(cfg->cmplt_pin);
  if(retval != OK)
  {
    return retval;
  }
  retval = stm32_configgpio(cfg->led_red);
  if(retval != OK)
  {
    return retval;
  }
  retval = stm32_configgpio(cfg->led_green);
  return retval;
}

/************************************************************************************
 * Basic Functions
 ************************************************************************************/

/* State transition functions */
/***********/
static int motor_enter_uninit(FAR struct motor_priv_s* priv)
{
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);

  _info("Enter uninit\n");
  motor_servo_off(priv->cfg);
  if(priv->state == MOTOR_READY)
  {
    /* If motor was in ready state, then turn off PFM */
    motor_stop_pfm(priv);
  }
  motor_led_red_off(priv->cfg);
  motor_led_green_off(priv->cfg);
  flags = enter_critical_section();
  priv->state = MOTOR_UNINIT;
  leave_critical_section(flags);

  return OK;
}

/***********/
static int motor_enter_unavailable(FAR struct motor_priv_s* priv)
{
  irqstate_t flags;
  DEBUGASSERT(priv != NULL);
  
  _info("Enter unavailable\n");
  motor_servo_off(priv->cfg);
  if(priv->state == MOTOR_READY)
  {
    /* If motor was in ready state, then turn off PFM */
    motor_stop_pfm(priv);
  }
  motor_led_red_on(priv->cfg);
  motor_led_green_on(priv->cfg);
  flags = enter_critical_section();
  priv->state = MOTOR_UNAVAILABLE;
  leave_critical_section(flags);

  return OK;
}

/***********/
static int motor_enter_alarm(FAR struct motor_priv_s* priv)
{
   irqstate_t flags;
 DEBUGASSERT(priv != NULL);
  
  _info("Enter alarm : %d\n", priv->cfg->motor_id);
  motor_servo_off(priv->cfg);
  if(priv->state == MOTOR_READY)
  {
    /* If motor was in ready state, then turn off PFM */
    motor_stop_pfm(priv);
  }
  motor_led_red_on(priv->cfg);
  motor_led_green_off(priv->cfg);
  flags = enter_critical_section();
  priv->state = MOTOR_ALARM;
  leave_critical_section(flags);

  return OK;
}

/***********/
static int motor_enter_ready(FAR struct motor_priv_s* priv)
{
  irqstate_t flags;
  int retval;

  DEBUGASSERT(priv != NULL);

  if(priv->state == MOTOR_READY)
  {
    /* Motor is already initialized */
    return OK;
  }

  if((priv->state != MOTOR_UNINIT) && (priv->state != MOTOR_ALARM))
  {
    return -ENXIO;  /* Motor is unavailable */
  }

  /* Only the motor with UNITIT state can reach here */
  _info("Enter ready\n");

  /* Config GPIO pins */
  retval = motor_init_gpio(priv->cfg);
  if(retval != OK)
  {
    return retval;
  }

  /* Allocate PFM driver only if not already done yet */
  if(priv->pfm == NULL)
  {
    priv->pfm = stm32_pfm_init(priv->cfg->motor_id);
    if(priv->pfm == NULL)
    {
      return -ENXIO;
    }
  }
  else
  {
    motor_stop_pfm(priv);
  }

  //motor_servo_on(priv->cfg);
  motor_led_green_on(priv->cfg);
  motor_led_red_off(priv->cfg);

  flags = enter_critical_section();
  priv->state = MOTOR_READY;
  leave_critical_section(flags);

  return OK;
}

/* Other basic functions */


/************************************************************************************
 * Name: motor_emergency
 *
 * Description:
 *   Service routine to handle change from Emergency button.
 *
 ************************************************************************************/
static int motor_emergency(int irq, FAR void *context, FAR void *arg)
{
  int i;

  UNUSED(irq);
  UNUSED(context);
  UNUSED(arg);

  if(motor_emergency_stat())
  {
    _alert("EMERGENCY ASSERT!!!\n");
    /* Emergency button armed. Stop all motor and turn on all LEDs */
    for(i = 0;i < 6;i++)
    {
      motor_enter_unavailable(&(motor_priv[i]));
    }
  }
  else
  {
    _info("Emergency released\n");
    /* Emergency button unarmed. Release all unavailable motor */
    for(i = 0;i < 6;i++)
    {
      if(motor_priv[i].state == MOTOR_UNAVAILABLE)
      {
        motor_enter_uninit(&(motor_priv[i]));
      }
    }
  }
  return OK;
}

/************************************************************************************
 * Name: motor_alarm
 *
 * Description:
 *   Service routine to handle motor alarm signal.
 *     arg : pointer to the particular motor structure.
 *
 ************************************************************************************/
static int motor_alarm(int irq, FAR void *context, FAR void *arg)
{
  FAR struct motor_priv_s* priv = (FAR struct motor_priv_s*)arg;

  UNUSED(irq);
  UNUSED(context);

  if(motor_alarm_stat(priv->cfg))
    return(motor_enter_alarm(priv));
  else
    return(OK);
}

/************************************************************************************
 * Driver interface
 ************************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     motor_open(FAR struct file *filep);
static int     motor_close(FAR struct file *filep);
static ssize_t motor_read(FAR struct file *filep, FAR char *buffer, size_t buflen);
static ssize_t motor_write(FAR struct file *filep, FAR const char *buffer, size_t buflen);
static int     motor_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_motorops =
{
  motor_open,  /* open */
  motor_close, /* close */
  motor_read,  /* read */
  motor_write, /* write */
  NULL,        /* seek */
  motor_ioctl, /* ioctl */
  NULL         /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL      /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/************************************************************************************
 * Name: motor_open
 *
 * Description:
 *   This function is called whenever the motor device is opened.
 *
 ************************************************************************************/

static int motor_open(FAR struct file *filep)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct motor_priv_s    *motor = inode->i_private;
  uint32_t                    tmp;
  int                         ret;

  _info("crefs: %d\n", motor->crefs);

  /* Get exclusive access to the device structures */

  ret = nxsem_wait(&(motor->exclsem));
  if (ret < 0)
    {
      goto errout;
    }

  /* Increment the count of references to the device.  If this the first
   * time that the driver has been opened for this device, then initialize
   * the device.
   */

  tmp = motor->crefs + 1;
  if (tmp == 0)
    {
      /* More than 255 opens; uint8_t overflows to zero */

      ret = -EMFILE;
      goto errout_with_sem;
    }

  /* Check if this is the first time that the driver has been opened. */

  /*if (tmp == 1)
    {
      FAR struct qe_lowerhalf_s *lower = upper->lower;

      / * Yes.. perform one time hardware initialization. * /

      DEBUGASSERT(lower->ops->setup != NULL);
      _info("calling setup\n");

      ret = lower->ops->setup(lower);
      if (ret < 0)
        {
          goto errout_with_sem;
        }
    }*/

  /* Save the new open count on success */

  motor->crefs = tmp;
  ret = OK;

errout_with_sem:
  nxsem_post(&(motor->exclsem));

errout:
  return ret;
}

/************************************************************************************
 * Name: motor_close
 *
 * Description:
 *   This function is called when the motor device is closed.
 *
 ************************************************************************************/

static int motor_close(FAR struct file *filep)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct motor_priv_s    *motor = inode->i_private;
  int                         ret;

  _info("crefs: %d\n", motor->crefs);

  /* Get exclusive access to the device structures */

  ret = nxsem_wait(&(motor->exclsem));
  if (ret < 0)
    {
      goto errout;
    }

  /* Decrement the references to the driver.  If the reference count will
   * decrement to 0, then uninitialize the driver.
   */

  if (motor->crefs > 1)
    {
      motor->crefs--;
    }
  /*else
    {
      FAR struct qe_lowerhalf_s *lower = motor->lower;

      / * There are no more references to the port * /

      upper->crefs = 0;

      / * Disable the PWM device * /

      DEBUGASSERT(lower->ops->shutdown != NULL);
      _info("calling shutdown: %d\n");

      lower->ops->shutdown(lower);
    }
    */

  nxsem_post(&(motor->exclsem));
  ret = OK;

errout:
  return ret;
}

/************************************************************************************
 * Name: motor_read
 *
 * Description:
 *   Read rough motor status. The returned data is a byte, which each bit represents
 *   occurence of any error of each motor (LSB represents motor0). 0 means the 
 *   particular motor is normal (i.e., ready for operation). The MSB (i.e.bit8 
 *   represent the emergency status)
 *
 ************************************************************************************/

static ssize_t motor_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  uint8_t v;
  UNUSED(filep);
  UNUSED(buflen);

  if(motor_priv[0].state != MOTOR_READY)
    v = 1;
  else
    v = 0;

  for(int i=1;i <= 5;i++)
  {
    v <<= 1;
    if(motor_priv[i].state != MOTOR_READY)
    {
      v |= 1;
    }
  }

  if(motor_emergency_stat())
    v |= 0x80;

  *buffer = (char)v;

  return 1;
}

/************************************************************************************
 * Name: motor_write
 *
 * Description:
 *   A dummy write method.  This is provided only to satsify the VFS layer.
 *
 ************************************************************************************/

static ssize_t motor_write(FAR struct file *filep, FAR const char *buffer, size_t buflen)
{
  /* Return a failure */
  UNUSED(filep);
  UNUSED(buffer);
  UNUSED(buflen);

  return -EPERM;
}

/************************************************************************************
 * Name: motor_ioctl
 *
 * Description:
 *   The standard ioctl method.  This is where ALL of the motor work is done.
 *
 ************************************************************************************/

static int motor_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode          *inode = filep->f_inode;
  FAR struct motor_priv_s   *motor;
  int                        ret;

  motor = inode->i_private;
  DEBUGASSERT(motor != NULL);
  _info("cmd: %d arg: %ld\n", cmd, arg);

  /* Get exclusive access to the device structures */

  ret = nxsem_wait(&motor->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Handle built-in ioctl commands */

  switch (cmd)
    {
      case MOTOR_CMD_INIT:
        ret = motor_enter_ready(motor);
        break;

      case MOTOR_CMD_SVOFF:
        motor_servo_off(motor->cfg);
        ret = OK;
        break;

      case MOTOR_CMD_SVON:
        motor_servo_on(motor->cfg);
        ret = OK;
        break;

      case MOTOR_CMD_RUN:
        {
          FAR struct motor_run_param_t *motor_run_param = (FAR struct motor_run_param_t *)arg;
          if(motor_run_param->speed >= MOTOR_MIN_SPEED &&
             motor_run_param->speed <= MOTOR_MAX_SPEED)
          {
            _info("Run motor with speed %u\n", motor_run_param->speed);

            motor_stop_pfm(motor);
            if(motor_run_param->is_cw)
            {
              motor_set_cw(motor->cfg);
            }
            else
            {
              motor_set_ccw(motor->cfg);
            }
            motor_start_pfm(motor, motor_run_param->speed, motor_run_param->step);
            ret = OK;
          }
          else
          {
            _err("Unsupport RUN param with speed=%u\n", motor_run_param->speed);
            ret = -ENOTSUP;
          }
        }
        break;

      case MOTOR_CMD_STOP:
        motor_stop_pfm(motor);
        ret = OK;
        break;

      case MOTOR_CMD_CLR_ALARM:
        if(motor->state == MOTOR_ALARM)
        {
          motor_alarm_res(motor->cfg);
          motor_clr_devcount(motor->cfg);
          usleep(2500);
          if(motor_alarm_stat(motor->cfg))
          {
            _err("Alarm signal persists on motor %d\n", motor->cfg->motor_id);
            ret = -EPERM;
          }
          else
          {
            ret = motor_enter_ready(motor);
          }
        }
        break;

      case MOTOR_CMD_GET_STATE:
        *( (uint32_t*)arg ) = (uint32_t)(motor->state);
        ret = OK;
        break;

      case MOTOR_CMD_GET_STATUS:
        *( (uint32_t*)arg ) = 0;
        if(motor_complete_stat(motor->cfg))
        {
          *( (uint32_t*)arg ) |= 1;
        }
        *( (uint32_t*)arg ) <<= 1;
        if(motor_alarm_stat(motor->cfg))
        {
          *( (uint32_t*)arg ) |= 1;
        }
        *( (uint32_t*)arg ) <<= 1;
        if(motor_emergency_stat())
        {
          *( (uint32_t*)arg ) |= 1;
        }
        ret = OK;
        break;

      /* Any unrecognized IOCTL commands might be platform-specific ioctl commands */

      default:
        {
          _err("Unrecognized cmd: %d arg: %ld\n", cmd, arg);
          ret = -ENOTSUP;
        }
        break;
    }

  nxsem_post(&(motor->exclsem));
  return ret;
}


/************************************************************************************
 * Pubic Functions
 ************************************************************************************/

int motor_initialize()
{
  char devpath[15];

  /* 0. Configure Emergency signal */
  stm32_configgpio(GPIO_EMERGENGY_STOP);
  (void)stm32_gpiosetevent(GPIO_EMERGENGY_STOP, true, true,
                       false, motor_emergency, (void *)0);

  for(int i = 0;i < 6;i++)
  {
    _info("Initialize motor ID %d:..", i);
    /* 1. Init semaphore */ 
    nxsem_init(&(motor_priv[i].exclsem), 0, 1);  // Binary sem

    /* 2. Init GPIO */
    if( motor_init_gpio(motor_priv[i].cfg) != OK )
    {
      _err("Failed to init GPIO\n");
      continue;  /* Skip this motor */
    }

    /* 3. Allocate pfm */
    motor_priv[i].pfm = stm32_pfm_init((motor_priv[i].cfg)->motor_id);

    if(motor_priv[i].pfm != NULL)
    {
      /* If PFM allocation is success */
      if( motor_enter_ready(&(motor_priv[i])) == OK)
      {
        _info("Entering READY...");
        (void)stm32_gpiosetevent((motor_priv[i].cfg)->alrm_pin, true, true,
                        false, motor_alarm, (void *)(&(motor_priv[i])));
        snprintf(devpath, 14, "/dev/motor%d", i);
        if(register_driver(devpath, &g_motorops, 0666, &(motor_priv[i])) != OK)
           _err("Unable to register %s\n", devpath);
        _info("Done\n");
      }
      else
      {
        motor_priv[i].state = MOTOR_UNAVAILABLE;
        _err("Failed to enter READY\n");
      }
    }
    else
    {
      motor_priv[i].state = MOTOR_UNAVAILABLE;
      _err("Failed to allocate PFM\n");
    }
  }

  /* Finally, check the emergency signal */
  if(motor_emergency_stat())
  {
    _alert("EMERGENCY ASSERT!!!\n");
    /* Emergency button armed. Stop all motor and turn on all LEDs */
    for(int i = 0;i < 6;i++)
    {
      motor_enter_unavailable(&(motor_priv[i]));
    }
  }

  return OK;
}
