/************************************************************************************
 * configs/coordinator/home.c
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
#include "stm32_tim.h"

/* This module utilizes Timer6 (a basic timer) */
#ifdef CONFIG_STM32F7_TIM6

#define TIMER_FREQ      100000
#define COUNTER_PERIOD  100

/************************************************************************************
 * Private Types
 ************************************************************************************/
struct s_home_data_t
{
    uint8_t u8_debounce_tmp;
    bool    b_home_status;
};

/************************************************************************************
 * Private Data
 ************************************************************************************/
static struct s_home_data_t s_home_data[6] =
{
    { .u8_debounce_tmp = 0, .b_home_status = false },
    { .u8_debounce_tmp = 0, .b_home_status = false },
    { .u8_debounce_tmp = 0, .b_home_status = false },
    { .u8_debounce_tmp = 0, .b_home_status = false },
    { .u8_debounce_tmp = 0, .b_home_status = false },
    { .u8_debounce_tmp = 0, .b_home_status = false }
};

static const uint32_t u32_home_gpio[6] =
{
    GPIO_HOME_0, GPIO_HOME_1, GPIO_HOME_2, GPIO_HOME_3, GPIO_HOME_4, GPIO_HOME_5
};

static struct stm32_tim_dev_s *s_timer;

/************************************************************************************
 * Name: tim_event
 *
 * Description:
 *   Service routine to handle timer event.
 *     arg : pointer to the particular motor structure.
 *
 ************************************************************************************/
static int tim_event(int irq, FAR void *context, FAR void *arg)
{
    int i;

    STM32_TIM_ACKINT(s_timer, 0);

    /* Read GPIO and feed to debouncing filter */
    for(i = 0;i < 6;i++)
    {
        s_home_data[i].u8_debounce_tmp <<= 1;
        if(stm32_gpioread(u32_home_gpio[i]))
            s_home_data[i].u8_debounce_tmp |= 1;
    }

    /* Process status after debouncing */
    for(i = 0;i < 6;i++)
    {
        switch (s_home_data[i].u8_debounce_tmp)
        {
            case 0:
                s_home_data[i].b_home_status = false;
                break;
            case 0xFF:
                s_home_data[i].b_home_status = true;
                break;
        }
    }

    return OK;
}

/************************************************************************************
 * Driver interface
 ************************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     home_open(FAR struct file *filep);
static int     home_close(FAR struct file *filep);
static ssize_t home_read(FAR struct file *filep, FAR char *buffer, size_t buflen);
static ssize_t home_write(FAR struct file *filep, FAR const char *buffer, size_t buflen);
static int     home_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_homeops =
{
    home_open,  /* open */
    home_close, /* close */
    home_read,  /* read */
    home_write, /* write */
    NULL,       /* seek */
    home_ioctl, /* ioctl */
    NULL        /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/************************************************************************************
 * Name: home_open
 *
 * Description:
 *   This function is called whenever the motor device is opened.
 *
 ************************************************************************************/

static int home_open(FAR struct file *filep)
{
    /* Do nothing */

    return OK;
}

/************************************************************************************
 * Name: home_close
 *
 * Description:
 *   This function is called when the motor device is closed.
 *
 ************************************************************************************/

static int home_close(FAR struct file *filep)
{
    /* Do nothing */

    return OK;
}

/************************************************************************************
 * Name: home_read
 *
 * Description:
 *   Read status of all home sensors in bit mapped. All sensor logics at GPIO
 *   pins are active-low. This function negates them to active-high.
 *
 ************************************************************************************/

static ssize_t home_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  char v;

  if(!(s_home_data[0].b_home_status))
    v = 1;
  else
    v = 0;

  for(int i=1;i <= 5;i++)
  {
    v <<= 1;
    if(!(s_home_data[i].b_home_status))
    {
      v |= 1;
    }
  }

  *buffer = v;

  return 1;
}

/************************************************************************************
 * Name: home_write
 *
 * Description:
 *   A dummy write method.  This is provided only to satsify the VFS layer.
 *
 ************************************************************************************/

static ssize_t home_write(FAR struct file *filep, FAR const char *buffer, size_t buflen)
{
    /* Return a failure */

     return -EPERM;
}

/************************************************************************************
 * Name: home_ioctl
 *
 * Description:
 *   The standard ioctl method.  No functionallity here.
 *
 ************************************************************************************/

static int home_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
    /* No IOCTL supported */

    return -ENOTSUP;
}

/************************************************************************************
 * Pubic Functions
 ************************************************************************************/

int home_initialize(void)
{
    int retval;

    /* 1. Config GPIO */
    for(int i = 0;i < 6;i++)
    {
        retval = stm32_configgpio(u32_home_gpio[i]);
        if(retval != OK)
        {
            _err("Unable to config GPIO for Home%d\n", i);
            return(retval);
        }
    }

    /* 2. Config timer */
    s_timer = stm32_tim_init(6);
    retval = STM32_TIM_SETCLOCK(s_timer, TIMER_FREQ);
    if(retval <= 0)
    {
        _err("Unable to set Timer6 to freqency %dHz\n", TIMER_FREQ);
        return -EINVAL;
    }
    STM32_TIM_SETPERIOD(s_timer, COUNTER_PERIOD);
    (void)STM32_TIM_SETMODE(s_timer, STM32_TIM_MODE_UP);

    /* 3. Attach timer interrupt */
    retval = STM32_TIM_SETISR(s_timer, tim_event, NULL, 0);
    if(retval != OK)
    {
        _err("Unable to bind Timer6 interrupt\n");
        (void)STM32_TIM_SETMODE(s_timer, STM32_TIM_MODE_DISABLED);
        return(retval);
    }
    STM32_TIM_ENABLEINT(s_timer, 0);

    /* 4. Register /dev/home */
    retval = register_driver("/dev/home", &g_homeops, 0666, NULL);
    if(retval != OK)
        _err("Unable to register /dev/home\n");

    return(retval);
}

#endif /* CONFIG_STM32F7_TIM6 */