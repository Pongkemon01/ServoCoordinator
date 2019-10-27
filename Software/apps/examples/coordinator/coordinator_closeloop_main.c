/****************************************************************************
 * examples/hello/coordinator_closeloop_main.c
 *
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <debug.h>

#include <nuttx/sensors/qencoder.h>
#include "driver/motor.h"
#include "driver/imu.h"

#include "geometry.h"

/****************************************************************************
 * Constant
 ****************************************************************************/
#define F_PI        3.1415926535897932384626433832795029f

/* The motor is configured to have 10000 pulse/rev. Its shaft is connected
to a 20:1 reducer gear. Hence, the full output resolution is 200000 pulse/rev.
The encoder equiped with the motot is configured to have the same value.
Therefore, we can imply that 1 QE plus equal to 1 motor step. */
#define QE_PER_REV      200000L     /* Total quadratic encoder count per rev */
#define QE_RESOLUTION   (2.0f * F_PI / (float)QE_PER_REV)

/****************************************************************************
 * Macro
 ****************************************************************************/
#define MOTOR_CMD(id,cmd,arg)   (ioctl(fdMotor[id], cmd, (unsigned long)arg))
#define MOTOR_INIT(id)          MOTOR_CMD(id,MOTOR_CMD_INIT,0)
#define MOTOR_SV_OFF(id)        MOTOR_CMD(id,MOTOR_CMD_SVOFF,0)
#define MOTOR_SV_ON(id)         MOTOR_CMD(id,MOTOR_CMD_SVON,0)
#define MOTOR_RUN(id,param_ptr) MOTOR_CMD(id,MOTOR_CMD_RUN,param_ptr)
#define MOTOR_STOP(id)          MOTOR_CMD(id,MOTOR_CMD_STOP,0)
#define MOTOR_CLR_ALARM(id)     MOTOR_CMD(id,MOTOR_CMD_CLR_ALARM,0)
#define MOTOR_GET_STATE(id,ret) MOTOR_CMD(id,MOTOR_CMD_GET_STATE,ret)
#define MOTOR_GET_CTRL(id,ret)  MOTOR_CMD(id,MOTOR_CMD_GET_STATUS,ret)

#define QE_CMD(id,cmd,arg)      (ioctl(fdQEncoder[id], cmd, (unsigned long)arg))
#define QE_GET_VALUE(id,ret)    QE_CMD(id,QEIOC_POSITION,ret)
#define QE_RESET(id)            QE_CMD(id,QEIOC_RESET,0)

#define IMU_CMD(cmd,arg)        (ioctl(fdIMU, cmd, (unsigned long)arg))
#define IMU_GET_SAMPLE_RATE(arg) IMU_CMD(IMU_CMD_GET_SAMPLE_RATE,arg)   /* Arg: Pointer to uint32_t */
#define IMU_GET_QUATERNION(arg) IMU_CMD(IMU_CMD_GET_QUATERNION,arg)     /* Arg: Array float[4] */
#define IMU_GET_LIN_ACCEL(arg)  IMU_CMD(IMU_CMD_GET_LIN_ACCEL,arg)      /* Arg: Array float[3] for x/y/z */
#define IMU_GET_LIN_VELO(arg)   IMU_CMD(IMU_CMD_GET_LIN_VELO,arg)       /* Arg: Array float[3] for x/y/z */
#define IMU_GET_LIN_DISP(arg)   IMU_CMD(IMU_CMD_GET_LIN_DISP,arg)       /* Arg: Array float[3] for x/y/z */

/* Writing a specific value to a specific place can cause a software reset 
 * Please refer to NVIC information of ARM Coretex-M7 datasheet
 */
#define SystemReset()   (*((uint32_t*)(0xE000ED0CUL))=(0x05FA0004UL))

/* File descriptors */
static int fdIMU;
static int fdQEncoder[6] = { 0, 0, 0, 0, 0, 0 };
static int fdMotor[6] = { 0, 0, 0, 0, 0, 0 };
static int fdHomeSensor = 0;

/* Previous quadrature encoder value */
static uint32_t u32QEVal[6];

/****************************************************************************
 * Open all devices. If the first attempt failed, the second is performed.
 * If even the second failed, then the function returns error.
 ****************************************************************************/
static int open_files(void)
{
    int retval = OK;
    char buf[60];

    /* Open all devices */
    for( int i = 0; i < 6; i++ )
    {
        sprintf( buf, "/dev/motor%d", i );
        fdMotor[ i ] = open( buf, O_RDWR );
        if( fdMotor[i] <= 0 )
        {
            _warn("Opening %s failed #1\n", buf);
            fdMotor[ i ] = open( buf, O_RDWR ); /* Attempt another open */
            if( fdMotor[i] <= 0 )
            {
                _err("Unable to open %s\n", buf);
                retval = -ENOTSUP;
            }

        }

        sprintf( buf, "/dev/qe%d", i );
        fdQEncoder[ i ] = open( buf, O_RDWR );
        if( fdQEncoder[i] <= 0 )
        {
            _warn("Opening %s failed #1\n", buf);
            fdQEncoder[ i ] = open( buf, O_RDWR ); /* Attempt another open */
            if( fdQEncoder[i] <= 0 )
            {
                _err("Unable to open %s\n", buf);
                retval = -ENOTSUP;
            }
        }
    }

    fdHomeSensor = open( "/dev/home", O_RDWR );
    if( fdHomeSensor <= 0 )
    {
        _warn("Opening HOME sensor failed #1\n");
        fdHomeSensor = open( "/dev/home", O_RDWR );
        if( fdHomeSensor <= 0 )
        {
            _err("Unable to open HOME sensor\n");
            retval = -ENOTSUP;
        }
    }

    fdIMU = open( "/dev/imu", O_RDWR );
    if( fdIMU <= 0 )
    {
        _warn("Opening IMU failed #1\n");
        fdIMU = open( "/dev/imu", O_RDWR );
        if( fdIMU <= 0 )
        {
            _err("Unable to open IMU\n");
            retval = -ENOTSUP;
        }
    }

    return retval;
}

void main_loop()
{
    float f[4];
    quaternion_t orientation;
    position_t displacement;
    position_t new_pos[6];

    /* Get orientation */
    IMU_GET_QUATERNION(f);
    orientation.w = f[0];
    orientation.x = f[1];
    orientation.y = f[2];
    orientation.z = f[3];

    /* Get linear displacement */
    IMU_GET_LIN_DISP(f);
    displacement.x = f[0];
    displacement.y = f[1];
    displacement.z = f[2];

    /* Generate compensated vectors */
    gen_compensate_pos(new_pos, &orientation, &displacement);

    /* Convert position into bisep angles */
    /* Convert angles into number of steps */    
}

/****************************************************************************
 * coordinator_main
 ****************************************************************************/

#if defined(BUILD_MODULE)
int main(int argc, FAR char *argv[])
#else
int coordinator_main(int argc, char *argv[])
#endif
{
    float f[4];
    quaternion_t orientation;
    position_t position;
    position_t new_pos[6];

    usleep(7000000);    /* Sleep 7 second for all hardware to initialized */

    if( open_files() != OK )
    {
        _err("Cannot fully initialize the system\n");
        usleep(3000000);
        SystemReset();
    }

    /* Initialize orientation */
    IMU_GET_QUATERNION(f);
    orientation.w = f[0];
    orientation.x = f[1];
    orientation.y = f[2];
    orientation.z = f[3];
    init_geometry(&orientation);

    /* Adjust the platform to home position */
    position.x = 0;     /* We don't have home sensor. Thus, all disp = 0 */
    position.y = 0;
    position.z = 0;
    gen_compensate_pos(new_pos, &orientation, &position);
    /* Convert position into bisep angles */
    /* Convert angles into number of steps */    
    /* Issue commands */

    /* Read current QE value */
    QE_GET_VALUE(0, &(u32QEVal[0]));
    QE_GET_VALUE(1, &(u32QEVal[1]));
    QE_GET_VALUE(2, &(u32QEVal[2]));
    QE_GET_VALUE(3, &(u32QEVal[3]));
    QE_GET_VALUE(4, &(u32QEVal[4]));
    QE_GET_VALUE(5, &(u32QEVal[5]));

    /* Main Loop, the only infinite loop */
    while(1)
    {
        main_loop();
        usleep(40000);
    }
}
