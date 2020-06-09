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
 * Driver for motor. There are 6 motors in this system. Each motor utilizes the same
 * driver module. Each motor is drived by a NIDEC-SANKYO S-FLAG servo driver.
 * The driver operates in "position mode" with its interal feedback loop.
 * External motor control should be perform in open-loop fashion because adding more
 * control loop is considered redundant and makes the overall system unstable through
 * out the operation load. The control signals used to interface with the S-FLAG are:
 *   - CMD_PLS - (output to driver) Pulse command to specify the rotational distance
 *   - CMD_DIR - (output to driver) Rotational direction (clock-wise/counter clock-wise)
 *   - SVON - (output to driver) Enable motor driver
 *   - RESET - (output to driver) Pulse to reset errors
 *   - PCLR - (output to driver) Deviation counter clear
 *   - ALM - (input from driver) Alarm status
 *   - POSIN - (input from driver) Positioning completed
 * All above signals except CMD_PLS are controlled directly from this driver via
 * GPIOs. The CMD_PLS signals are controlled separately by PFM driver, which is
 * an internal driver in the kernel space (See stm32_pfm.c and stm32_pfm.h).
 * Each motor also has 2 LEDs (Red and Green) to indicates the status as:
 *    - Red-on, Green-on - Motor is not ready or uninitialized
 *    - Red-off, Green-on - Motor is initialized and ready
 *    - Red-on, Green-off - Motor alarm
 *    - Red-off, Green-off - Driver is uninitailized
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
  volatile int32_t            step_count;
  volatile motor_state_t      state;
  volatile bool               is_cw;  /* Is negative direction? */
  volatile bool               is_start; /* The motor has been started without any stop command */
};

/************************************************************************************
 * Private Data
 ************************************************************************************/

// GPIO assignment for each motor signal
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

// Motor internal data (including the previous defined GPIO configuration)
struct motor_priv_s motor_priv[6] =
{

{
  .cfg    = &(motor_cfg[0]),
  .pfm    = NULL,
  .step_count = 0L,
  .is_cw  = false,
  .is_start = false,
  .state  = MOTOR_UNINIT
},

{
  .cfg    = &(motor_cfg[1]),
  .pfm    = NULL,
  .step_count = 0L,
  .is_cw  = false,
  .is_start = false,
  .state  = MOTOR_UNINIT
},

{
  .cfg    = &(motor_cfg[2]),
  .pfm    = NULL,
  .step_count = 0L,
  .is_cw  = false,
  .is_start = false,
  .state  = MOTOR_UNINIT
},

{
  .cfg    = &(motor_cfg[3]),
  .pfm    = NULL,
  .step_count = 0L,
  .is_cw  = false,
  .is_start = false,
  .state  = MOTOR_UNINIT
},

{
  .cfg    = &(motor_cfg[4]),
  .pfm    = NULL,
  .step_count = 0L,
  .is_cw  = false,
  .is_start = false,
  .state  = MOTOR_UNINIT
},

{
  .cfg    = &(motor_cfg[5]),
  .pfm    = NULL,
  .step_count = 0L,
  .is_cw  = false,
  .is_start = false,
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

#define motor_get_pfm_count(motor)  (motor->pfm->ops->get_count(motor->pfm))
#define motor_stop_pfm(motor)       (motor->pfm->ops->stop(motor->pfm))
#define motor_is_idle(motor)        (motor->pfm->ops->is_idle(motor->pfm))
#define motor_start_pfm(motor,p,c)  (motor->pfm->ops->start(motor->pfm,p,c))

/************************************************************************************
 * Low-level Functions
 ************************************************************************************/

static inline void motor_alarm_res(FAR struct motor_cfg_s* cfg)
{
  /* Sending a pulse to reset alarm status */
  motor_setpin(cfg->alrm_res_pin);
  usleep(25000);      /* The datasheet specified 25mS(min) */
  motor_respin(cfg->alrm_res_pin);
}

static inline void motor_clr_devcount(FAR struct motor_cfg_s* cfg)
{
  /* Sending a pulse to clear deviation counter */
  motor_setpin(cfg->dev_clr_pin);
  usleep(25000);      /* The datasheet specified 25mS(min) */
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
/* The driver has 4 states:
 *  - Uninitialized - The motor is not yet initialized
 *  - Unavailable - The motor has been initialized but is in "stop" mode
 *  - Alarm - The ALM signal from S-FLAG is set
 *  - Ready - The motor is ready to operate
 * At power-on, the driver is started within the state "Unintialized". Then, if all 
 * initialization steps are success, the state changes to "Ready". The ALRM signal
 * and EMERGENCY switch are hooked to GPIOs with initerrupt-on-change mode.
 * If the ALRM of a motor is asserted, the corresponding motor driver goes into "Alarm".
 * If the EMERGENCY switch is pushed (on), all motor drivers gos into "Unavailable".
 * The Alarm state can be cleared through the software, which, if success, put the
 * corresponding motor back to "Ready" state. However, the "Unavailable" state must
 * be cleared by releasing the EMERGENCY button. Clearing Unavailable state puts the
 * driver into "Uninitialzed" state that requires the full initialization steps again.
 * 
 * Transitioning from a state into another requires some operations. The following 
 * funtions are the operations needed to perform before entering each particular state.
/***********/
static int motor_enter_uninit(FAR struct motor_priv_s* priv)
{
  irqstate_t flags;
  uint32_t l;

  DEBUGASSERT(priv != NULL);

  _info("Enter uninit\n");
  motor_servo_off(priv->cfg);
  if(priv->state == MOTOR_READY)
  {
    /* If motor was in ready state, then turn off PFM */
     if(priv->is_start)
    {
      l = motor_stop_pfm(priv);
      if(priv->is_cw)
        priv->step_count -= l;
      else
        priv->step_count += l;
      priv->is_start = false;
    }
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
  uint32_t  l;
  
  _info("Enter unavailable\n");
  motor_servo_off(priv->cfg);
  if(priv->state == MOTOR_READY)
  {
    /* If motor was in ready state, then turn off PFM */
    if(priv->is_start)
    {
      l = motor_stop_pfm(priv);
      if(priv->is_cw)
        priv->step_count -= l;
      else
        priv->step_count += l;
      priv->is_start = false;
    }
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
  uint32_t l;
  
  _info("Enter alarm : %d\n", priv->cfg->motor_id);
  motor_servo_off(priv->cfg);
  if(priv->state == MOTOR_READY)
  {
    /* If motor was in ready state, then turn off PFM */
    if(priv->is_start)
    {
      l = motor_stop_pfm(priv);
      if(priv->is_cw)
        priv->step_count -= l;
      else
        priv->step_count += l;
      priv->is_start = false;
    }
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
    (void)motor_stop_pfm(priv);
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
 *   Service routine to handle status of Emergency button.
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
  UNUSED(filep);
  return OK;
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
  UNUSED(filep);
  return OK;
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
  uint32_t                   l;

  motor = inode->i_private;
  DEBUGASSERT(motor != NULL);

  /* Handle built-in ioctl commands */

  switch (cmd)
    {
      case MOTOR_CMD_INIT:
        ret = motor_enter_ready(motor);
        break;

      case MOTOR_CMD_SVOFF:
        //_info("MOTOR %u OFF\n", motor->cfg->motor_id);
        motor_servo_off(motor->cfg);
        ret = OK;
        break;

      case MOTOR_CMD_SVON:
        //_info("MOTOR %u ON\n", motor->cfg->motor_id);
        motor_servo_on(motor->cfg);
        ret = OK;
        break;

      case MOTOR_CMD_RUN:
        {
          FAR struct motor_run_param_t *motor_run_param = (FAR struct motor_run_param_t *)arg;
          if(motor_run_param->speed >= MOTOR_MIN_SPEED &&
             motor_run_param->speed <= MOTOR_MAX_SPEED)
          {
            //_info("Run motor %u with speed %u and steps %d\n", motor->cfg->motor_id, motor_run_param->speed, motor_run_param->step);

            if( motor->is_start )
            {
              l = motor_stop_pfm(motor);
              if(motor->is_cw)
                motor->step_count -= l;
              else
                motor->step_count += l;
            }

            /* Counter-clockwise is considered as positive direction in this platform */
            if(motor_run_param->step < 0)
            {
              motor->is_cw = true;
              motor_set_cw(motor->cfg);
              motor_start_pfm(motor, motor_run_param->speed, (uint32_t)(-(motor_run_param->step)));
            }
            else
            {
              motor->is_cw = false;
              motor_set_ccw(motor->cfg);
              motor_start_pfm(motor, motor_run_param->speed, (uint32_t)(motor_run_param->step));
            }
            motor->is_start = true;

            ret = OK;
          }
          else
          {
            _err("Unsupport RUN param with step=%d speed=%u\n", motor_run_param->step, motor_run_param->speed);
            ret = -ENOTSUP;
          }
        }
        break;

      case MOTOR_CMD_STOP:
        l = motor_stop_pfm(motor);
        if(motor->is_cw)
          motor->step_count -= l;
        else
          motor->step_count += l;
        motor->is_start = false;
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
        *( (motor_state_t*)arg ) = motor->state;
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

      case MOTOR_CMD_GET_COUNTER:
        if(!(motor->is_start))
          *( (int32_t*)arg ) = motor->step_count;
        else
        {
          if(motor->is_cw)
            *( (int32_t*)arg ) = motor->step_count - motor_get_pfm_count(motor);
          else
            *( (int32_t*)arg ) = motor->step_count + motor_get_pfm_count(motor);
        }
        ret = OK;
        break;

      case MOTOR_CMD_IS_RUNNING:
        if(motor_is_idle(motor))
          *( (uint32_t*)arg ) = 0;
        else
          *( (uint32_t*)arg ) = 1;
        ret = OK;
        break;

      case MOTOR_CMD_RES_COUNTER:
        motor->step_count = 0;
        ret = OK;
        break;

      case MOTOR_CMD_SET_COUNTER:
        motor->step_count = *( (int32_t*)arg );
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

    /* 1. Init GPIO */
    if( motor_init_gpio(motor_priv[i].cfg) != OK )
    {
      _err("Failed to init GPIO\n");
      continue;  /* Skip this motor */
    }

    /* 2. Allocate pfm driver */
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
