/****************************************************************************
 * examples/hello/coordinator_motor.c
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
#include <math.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <debug.h>
#include <signal.h>
#include <pthread.h>
#include <semaphore.h>
#include <time.h>

#include <nuttx/sensors/qencoder.h>
#include "driver/motor.h"

/****************************************************************************
 * Macro
 ****************************************************************************/
#define MOTOR_CMD(fd,cmd,arg)   (ioctl(fd, cmd, (unsigned long)arg))
#define MOTOR_INIT(fd)          MOTOR_CMD(fd,MOTOR_CMD_INIT,0)
#define MOTOR_SV_OFF(fd)        MOTOR_CMD(fd,MOTOR_CMD_SVOFF,0)
#define MOTOR_SV_ON(fd)         MOTOR_CMD(fd,MOTOR_CMD_SVON,0)
#define MOTOR_RUN(fd,param_ptr) MOTOR_CMD(fd,MOTOR_CMD_RUN,param_ptr)
#define MOTOR_STOP(fd)          MOTOR_CMD(fd,MOTOR_CMD_STOP,0)
#define MOTOR_CLR_ALARM(fd)     MOTOR_CMD(fd,MOTOR_CMD_CLR_ALARM,0)
#define MOTOR_GET_STATE(fd,ret) MOTOR_CMD(fd,MOTOR_CMD_GET_STATE,ret)
#define MOTOR_GET_CTRL(fd,ret)  MOTOR_CMD(fd,MOTOR_CMD_GET_STATUS,ret)
#define MOTOR_GET_COUNTER(fd,ret) MOTOR_CMD(fd,MOTOR_CMD_GET_COUNTER,ret)

#define QE_CMD(fd,cmd,arg)      (ioctl(fd, cmd, (unsigned long)arg))
#define QE_GET_VALUE(fd,ret)    QE_CMD(fd,QEIOC_POSITION,ret)
#define QE_RESET(fd)            QE_CMD(fd,QEIOC_RESET,0)

/* The motor is configured to have 10000 pulse/rev. Its shaft is connected
to a 20:1 reducer gear. Hence, the full output resolution is 200000 pulse/rev.
The encoder equiped with the motot is configured to have the same value.
Therefore, we can imply that 1 QE pulse equal to 1 motor step. */
#define STEP_PER_REV      14000L     /* Total motor steps per rev */
#define MAX_MOTOR_SPEED     10000L    /* Maximum step/second */

/* PID coefficients */
#define KP      (3.5f)
#define KI      (0.15f)
#define KD      (7.0f)
#define INT_LIM (1000.0f)
#define D_LPFA  (0.2846f)
#define D_LPFB  (1.0f - D_LPFA)

/* These global variables are here for tuning */
//sem_t kSem;
float f32KP = KP;
float f32KI = KI;
float f32KD = KD;

/************************************************************************************
 * Private Functions
 ************************************************************************************/
/* Helper function to represent "millis" function in Arduino system */
static uint32_t millis(void)
{
    struct timespec tp;

    if (clock_gettime(CLOCK_MONOTONIC, &tp)) 
    {
        return 0;
    }

    return ( ( ( (uint32_t)tp.tv_sec ) * 1000U ) + ( (uint32_t)tp.tv_nsec / 1000000U ) );
}

/****************************************************************************
 * motor_thread: the thread that manage a specified motor position.
 * Parameters:
 *    iMotorFile = file descripter to the specified motor
 *    iQEFile = file descripter to the encoder attach to the motor iMotorFile
 *    iHomeFile = file descripter for common Home sensor
 *    theta = the set-point of the motor (aka. expected absolute rotational position)
 *    theta_sem = semaphore that controls "theta" access
 ****************************************************************************/
typedef struct 
{
    uint32_t id;

    /* File descripters */
    int iMotorFile;
    int iQEFile;
    int iHomeFile;

    /* Motor set-point and its semaphore */
    sem_t theta_sem;
    int32_t theta;
}motor_thread_param_t;

// Info if change from pthread to task
// typedef CODE int (*main_t)(int argc, FAR char *argv[]);

void* motor_thread( void* arg )
{
    motor_thread_param_t* param;
    struct motor_run_param_t  m_param;
    int32_t set_point_pos, cur_pos;
    uint8_t home_mask;
    motor_state_t state;
    uint32_t speed;
    int32_t rel_step;

    float ITerm, DTerm, err, prev_cur_pos, u;

    DEBUGASSERT(arg != NULL);
    param = (motor_thread_param_t*)arg;
    DEBUGASSERT(param->id <= 5);

    _info("Start motor thread for motor %d\n", param->id);
    ITerm = 0;
    DTerm = 0;
    prev_cur_pos = 0;
    home_mask = ((uint8_t)(1) << (param->id));
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    QE_RESET(param->iQEFile);

    while(1)
    {
        /* Translate Theta from radian to motor steps */
        DEBUGASSERT( sem_wait( &(param->theta_sem) ) == OK );
        set_point_pos = param->theta;
        DEBUGASSERT( sem_post( &(param->theta_sem) ) == OK );

        /* Read current position from QE */
        QE_GET_VALUE( param->iQEFile, &cur_pos );
    
        /* Compute difference between current and setpoint */
        rel_step = ( set_point_pos - cur_pos ) % STEP_PER_REV; /* We move only within 1 rev */
        //_info("Set point %ld, current %ld, error %ld\n", set_point_pos, cur_pos, rel_step);
        err = (float)rel_step;

        /* PID */

        /* Integral term with limit capped */
        ITerm += ( f32KI * err );
        if( ITerm > INT_LIM )
        {
            ITerm = INT_LIM;
        }
        else
        {
            if( ITerm < -INT_LIM )
                ITerm = -INT_LIM;
        }

        /* Derivative term */
        DTerm = ( D_LPFA * DTerm ) - ( D_LPFB * f32KD * ((float)cur_pos - prev_cur_pos ) );
        prev_cur_pos = (float)cur_pos;

        /* Combine all into final PID result */
        u = (f32KP * err) + ITerm + DTerm;
        
        /* Convert control result into speed */
        if(rel_step > 400)
            rel_step = 400;
        else
            if( rel_step < -400 )
                rel_step = -400;
        
        speed = (uint32_t)fabs(u);
        if( speed > MAX_MOTOR_SPEED )
            speed = MAX_MOTOR_SPEED;

        if( speed > 0 && rel_step != 0 )
        {
            /* Command the motor */
            m_param.step = rel_step;
            m_param.speed = speed;

            //_info("Move motor %d : %d steps with speed %u\n", param->id, m_param.step, m_param.speed);

            /* Check wheter motor is ready */
            MOTOR_GET_STATE( param->iMotorFile, &state );
            for(int count = 3; (count > 0) && (state != MOTOR_READY); count--)
            {
                MOTOR_CLR_ALARM( param->iMotorFile );
                MOTOR_GET_STATE( param->iMotorFile, &state );
            }

            if( state != MOTOR_READY )
            {
                _err( "Motor %d is not ready\n", param->id );
            }
            else
            {
                /* Run the motor */
                if( MOTOR_RUN( param->iMotorFile, &m_param ) != OK )
                {
                    _err( "Motor %d failed to step\n", param->id );
                }
            }
        }

        usleep(20000);  /* Sleep for 40ms */
    }
}
