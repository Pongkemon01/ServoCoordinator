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
 * Driver for home sensor. Each motor is equiped with a home sensor located at
 * the lowest arm position. Each sensor operates as a on/off switch with normally off
 * convension. This driver periodically read the status of physical switches,
 * debounce them, and keep the current statuses for application module.  
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/wqueue.h>

#include <sys/types.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>
#include <time.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_internal.h"
#include "up_arch.h"
#include "stm32_gpio.h"

#define DEBOUNCE_FREQ   (100)
#define DEBOUNCE_DELAY  (1000000/(CONFIG_USEC_PER_TICK * DEBOUNCE_FREQ))

#ifndef CONFIG_SCHED_LPWORK
#  error "HOME sensor requires a low-priority work queue"
#endif

#define TASK_QUEUE  LPWORK

/************************************************************************************
 * Private Types
 ************************************************************************************/
struct s_home_data_t
{
    uint8_t u8_debounce_tmp;    // Temporary data for debouncing
    bool    b_home_status;      // Current switch status
};

/************************************************************************************
 * Private Data
 ************************************************************************************/
// Operating data of each switch with corresponding initialization
static struct s_home_data_t s_home_data[6] =
{
    { .u8_debounce_tmp = 0, .b_home_status = false },
    { .u8_debounce_tmp = 0, .b_home_status = false },
    { .u8_debounce_tmp = 0, .b_home_status = false },
    { .u8_debounce_tmp = 0, .b_home_status = false },
    { .u8_debounce_tmp = 0, .b_home_status = false },
    { .u8_debounce_tmp = 0, .b_home_status = false }
};

// GPIO number attached to each Home switch
static const uint32_t u32_home_gpio[6] =
{
    GPIO_HOME_0, GPIO_HOME_1, GPIO_HOME_2, GPIO_HOME_3, GPIO_HOME_4, GPIO_HOME_5
};

// Periodic worker queue instance
static struct work_s worker;

/************************************************************************************
 * Name: tim_event
 *
 * Description:
 *   Service routine periodically called by periodic-worker queue.
 *     arg : pointer to the particular motor structure. (Predefined in initialization)
 *
 ************************************************************************************/
static void tim_event(FAR void *arg)
{
    int i;

    UNUSED(arg);

    /* 
     Debouncing process is simple. This routine reads each switch status
     and keep it in a temporary variable as a bit (0 = open, 1 = close).
     If a particular switch status has been persisted for 8 previous reading
     (i.e., the value in its temporary variable is either 0 (all 0) or 0xFF 
     (all 1)), the actual switch status is changed to follow the reading.
     */

    /* Read GPIO and feed to debouncing filter */
    for(i = 0;i < 6;i++)
    {
        s_home_data[i].u8_debounce_tmp <<= 1;
        if(stm32_gpioread(u32_home_gpio[i]))
            s_home_data[i].u8_debounce_tmp |= 1;
    }

    /* Send logging */
    _info( ":H;%lu;%02X;%02X;%02X;%02X;%02X;%02X\n", clock(), s_home_data[0].u8_debounce_tmp, 
        s_home_data[1].u8_debounce_tmp, s_home_data[2].u8_debounce_tmp, 
        s_home_data[3].u8_debounce_tmp, s_home_data[4].u8_debounce_tmp, 
        s_home_data[5].u8_debounce_tmp );

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

    /* The periodic task must re-schedule itself into the system worker queue *./
    /* Re-enable timer event */
    if(work_queue(TASK_QUEUE, &worker, tim_event, NULL, DEBOUNCE_DELAY) != OK)
    {
        _err("Unable to register HOME sensor to work queue\n");
    }
}

/************************************************************************************
 * Driver interface to compliant with POSIX open/close/read/write/ioctl file operation
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
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL      /* unlink */
#endif
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
    UNUSED(filep);

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
    UNUSED(filep);

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

    UNUSED(filep);
    UNUSED(buflen);

    v = 0;

    // Convert statuses of separated switch into a bit field
    for(int i=5;i >= 0;i--)
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
    UNUSED(filep);
    UNUSED(buffer);
    UNUSED(buflen);

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
    UNUSED(filep);
    UNUSED(cmd);
    UNUSED(arg);

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

    /* 2. Register work queue for periodic task */
    retval = work_queue(TASK_QUEUE, &worker, tim_event, NULL, DEBOUNCE_DELAY);
    if(retval != OK)
    {
        _err("Unable to register HOME sensor to work queue\n");
        return retval;
    }

    /* 4. Register /dev/home */
    retval = register_driver("/dev/home", &g_homeops, 0666, NULL);
    if(retval != OK)
        _err("Unable to register /dev/home\n");

    return(retval);
}
