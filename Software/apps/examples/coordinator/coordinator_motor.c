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
#define MOTOR_RES_COUNTER(fd)    MOTOR_CMD(fd,MOTOR_CMD_RES_COUNTER,0)
#define MOTOR_SET_COUNTER(fd,s)  MOTOR_CMD(fd,MOTOR_CMD_SET_COUNTER,s)
#define MOTOR_IS_RUNNING(fd,ret) MOTOR_CMD(fd,MOTOR_CMD_IS_RUNNING,ret)

#define QE_CMD(fd,cmd,arg)      (ioctl(fd, cmd, (unsigned long)arg))
#define QE_GET_VALUE(fd,ret)    QE_CMD(fd,QEIOC_POSITION,ret)
#define QE_RESET(fd)            QE_CMD(fd,QEIOC_RESET,0)

/* The motor is configured to have 10000 pulse/rev. Its shaft is connected
to a 20:1 reducer gear. Hence, the full output resolution is 200000 pulse/rev.
The encoder equiped with the motot is configured to have the same value.
Therefore, we can imply that 1 QE pulse equal to 1 motor step. */
#define STEP_PER_REV      14000L     /* Total motor steps per rev */

#define DELTA_T            20000UL  /* Loop time */

#define HOME_OFFSET        (-3000L)    /* Step offset between home sensor and horizontal point */

// The encoder is not reliable enough!!! We should count every thing
// and pretend that the internal counting is real.

/* PID coefficients */
#define KP      (2.0f)
#define KI      (0.7f)
#define KD      (2.0f)
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

/****************************************************************************
 * Private function
 * Negative rel_step means CW.
 * Positive rel_step means CCW.
 ****************************************************************************/
static bool run_motor( motor_thread_param_t* param, int32_t rel_step, uint32_t speed )
{
    struct motor_run_param_t  m_param;
    motor_state_t state;

    /* Command the motor */
    m_param.step = rel_step;
    m_param.speed = speed;

    /* Check wheter motor is ready */
    //_info("Run motor %d with step %d speed %d\n", param->id, m_param.step, m_param.speed);
    MOTOR_GET_STATE( param->iMotorFile, &state );
    if( state != MOTOR_READY )
    {
        for(int count = 3; (count > 0) && (state != MOTOR_READY); count--)
        {
            MOTOR_CLR_ALARM( param->iMotorFile );
            MOTOR_GET_STATE( param->iMotorFile, &state );
        }
    }

    /* If motor is still not ready */
    if( state != MOTOR_READY )
    {
        _err( "Motor %d is not ready\n", param->id );
        return false;
    }
    else
    {
        /* Run the motor */
        if( MOTOR_RUN( param->iMotorFile, &m_param ) != OK )
        {
            _err( "Motor %d failed to step\n", param->id );
            return false;
        }
    }
    return true;
}

/****************************************************************************
 * Public function
 ****************************************************************************/
void* motor_thread( void* arg )
{
    motor_thread_param_t* param;
    int32_t set_point_pos, cur_pos;
    uint8_t home_mask, home_stat;
    uint32_t is_running;
    uint32_t speed;
    int32_t rel_step;

    float ITerm, DTerm, err, prev_cur_pos, u;

    DEBUGASSERT(arg != NULL);
    param = (motor_thread_param_t*)arg;
    DEBUGASSERT(param->id <= 5);

    home_mask = ((uint8_t)(1) << (param->id));
    MOTOR_SV_ON( param->iMotorFile );
    _info("Start motor thread for motor %d with home mask %u\n", param->id, home_mask);

    /* Reset motor position to home */
    // 1. Rotate positive direction (cw) until home sensor trigged.
    if( read( param->iHomeFile, &home_stat, 1 ) == 1 )
    {
        /* Home status: 0 = released, 1 = pressed */
        for(rel_step = 0; (rel_step < 13) && ((home_stat & home_mask) == 0); rel_step++ )
        {
            if( run_motor( param, -1200, 1000UL ) == false )
                break;
            do
            {
                usleep(1);  /* wait 1 quantum */
                if( read( param->iHomeFile, &home_stat, 1 ) == 1 )
                {
                    if((home_stat & home_mask) != 0)
                    {
                        MOTOR_STOP( param->iMotorFile );    /* Found Home */
                        break;
                    }
                }
                MOTOR_IS_RUNNING( param->iMotorFile, &is_running );
            }while( is_running != 0 );
        }

        if((home_stat & home_mask) != 0)
        {
            // 2. Rotate negative direction (ccw) slowly until home sensor released.
            for(rel_step = 0; ( rel_step < 36 ) && ((home_stat & home_mask) != 0); rel_step++ )
            {
                if( run_motor( param, 100, 1000UL ) == false )
                    break;
                do
                {
                    usleep(1);  /* wait 1 quantum */
                    if( read( param->iHomeFile, &home_stat, 1 ) == 1 )
                    {
                        if((home_stat & home_mask) == 0)
                        {
                            MOTOR_STOP( param->iMotorFile );    /* Home Released */
                            break;
                        }
                    }

                    MOTOR_IS_RUNNING( param->iMotorFile, &is_running );
                }while( is_running != 0 );
            }

            // 3. Rotate negative direction (ccw) for 3000 steps (approx.)
            if( run_motor( param, -HOME_OFFSET , 1000UL ) )
            {
                do
                {
                    usleep(1);  /* wait 1 quantum */
                    MOTOR_IS_RUNNING( param->iMotorFile, &is_running );
                }while( is_running != 0 );
            }
        }
        else
        {
            _warn( "Unable to find Home sensor for motor %d\n", param->id );
        }
    }
    else
    {
        _warn( "Unable to read Home sensor for motor %d\n", param->id );
    }

    // 4. Reset all counter at the current point
    ITerm = 0;
    DTerm = 0;
    prev_cur_pos = 0;
    cur_pos = 0;
    MOTOR_STOP( param->iMotorFile );
    MOTOR_RES_COUNTER( param->iMotorFile );
    QE_RESET( param->iQEFile );

    _info("Motor thread %d is ready.\n", param->id);

    while(1)
    {
        /* Theta (i.e.,the desired angle) is unit in motor absolute steps */
        DEBUGASSERT( sem_wait( &(param->theta_sem) ) == OK );
        set_point_pos = param->theta;
        DEBUGASSERT( sem_post( &(param->theta_sem) ) == OK );

        /* Read current position from motor */
        MOTOR_IS_RUNNING( param->iMotorFile, &is_running );
        if( is_running != 0 )
            MOTOR_STOP( param->iMotorFile );
        MOTOR_GET_COUNTER( param->iMotorFile, &cur_pos );

        /* Compensate drift by checking for home sensor */
        if( read( param->iHomeFile, &home_stat, 1 ) == 1 )
        {
            if( (home_stat & home_mask) != 0 )
            {
                /* If home is trigged, the motor is at home sensor position. Then, reset the position. */
                if( abs(cur_pos - HOME_OFFSET) > 500 )
                {
                    MOTOR_SET_COUNTER( param->iMotorFile, HOME_OFFSET );
                    cur_pos = HOME_OFFSET;
                }
            }
        }

        /* Compute difference between current and setpoint */
        rel_step = ( set_point_pos - cur_pos ) % STEP_PER_REV; /* We move only within 1 rev */
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
        if( speed > MOTOR_MAX_SPEED )
            speed = MOTOR_MAX_SPEED;

        /* Modifier for too slow movement */
        if( speed < MOTOR_MIN_SPEED )
        {
            if( rel_step < 0 )
                rel_step = -speed;
            else
                rel_step = speed;
            speed = MOTOR_MIN_SPEED;
        }

        if( speed > 0 && rel_step != 0 )
        {
            //_info("Set point %ld, current %ld, error %ld, speed %u\n", set_point_pos, cur_pos, rel_step, speed);
            run_motor( param, rel_step, speed );
        }

        usleep(DELTA_T);  /* Sleep for 20ms */
    }

    /* We should not reach here */
    _err( "Motor thread for motor %d terminated!!\n", param->id );
    MOTOR_SV_OFF( param->iMotorFile );
    return(0);
}
