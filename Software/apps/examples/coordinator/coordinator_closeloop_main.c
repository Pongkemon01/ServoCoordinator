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
#include <math.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <debug.h>
#include <signal.h>
#include <semaphore.h>

#include <nuttx/timers/timer.h>
#include <nuttx/sensors/qencoder.h>
#include "driver/motor.h"
#include "driver/imu.h"

#include "geometry.h"

/****************************************************************************
 * Constant
 ****************************************************************************/
#define F_PI        3.1415926535897932384626433832795029f

/* Ideal resolution is 14000 pluses/rev, equivalent to 0.104mm resolution
 * of the end-effector movement. It means 700 pulses/rev of motor with
 * 1:20 reducing gear. The maximum frequency of PFM is 10000Hz. which is
 * more than enough for -pi/2 - pi/2 operation range (half a circle).
 * 
 * *** Motor driver config Numerator=32768, Denominator=175 ***
 * 
 * Put parameters into https://www.marginallyclever.com/other/samples/fk-ik-test.html
 * to verify some design.
 * 
 */

/* The motor is configured to have 10000 pulse/rev. Its shaft is connected
to a 20:1 reducer gear. Hence, the full output resolution is 200000 pulse/rev.
The encoder equiped with the motot is configured to have the same value.
Therefore, we can imply that 1 QE pulse equal to 1 motor step. */
#define STEP_PER_REV      14000L     /* Total motor steps per rev */
#define ANGLE_TO_STEP_FACTOR   ((float)STEP_PER_REV / (2.0f * F_PI))
#define MAX_MOTOR_SPEED     10000L    /* Maximum step/second */

#define CONFIG_QUANTUM_STEP_MS  (40L) /* Interval of cloosed-loop control in us. (40ms) */
#define CONFIG_QUANTUM_STEP     ((uint32_t)(CONFIG_QUANTUM_STEP_MS * 1000L)) /* Inteval in us */
#define CONFIG_COMPLETION_TIME  (CONFIG_QUANTUM_STEP_MS - 5L) /* The time that all movement should finish */
#define MAX_MOTOR_STEP_PER_Q    (MAX_MOTOR_SPEED * CONFIG_COMPLETION_TIME / 1000L)
#define CONFIG_COORDINATOR_TIMER_SIGNO 17

/****************************************************************************
 * Macro
 ****************************************************************************/
#define QE_CMD(fd,cmd,arg)      (ioctl(fd, cmd, (unsigned long)arg))
#define QE_GET_VALUE(fd,ret)    QE_CMD(fd,QEIOC_POSITION,ret)
#define QE_RESET(fd)            QE_CMD(fd,QEIOC_RESET,0)

#define IMU_CMD(cmd,arg)        (ioctl(fdIMU, cmd, (unsigned long)arg))
#define IMU_GET_SAMPLE_RATE(arg) IMU_CMD(IMU_CMD_GET_SAMPLE_RATE,arg)   /* Arg: Pointer to uint32_t */
#define IMU_GET_QUATERNION(arg) IMU_CMD(IMU_CMD_GET_QUATERNION,arg)     /* Arg: Array float[4] */
#define IMU_GET_TILT(arg)       IMU_CMD(IMU_CMD_GET_TILT,arg)           /* Arg: Array float[3] for x/y/z */
#define IMU_GET_GRAVITY(arg)    IMU_CMD(IMU_CMD_GET_GRAVITY,arg)        /* Arg: Array float[3] for x/y/z */
#define IMU_GET_LIN_ACCEL(arg)  IMU_CMD(IMU_CMD_GET_LIN_ACCEL,arg)      /* Arg: Array float[3] for x/y/z */
#define IMU_GET_LIN_VELO(arg)   IMU_CMD(IMU_CMD_GET_LIN_VELO,arg)       /* Arg: Array float[3] for x/y/z */
#define IMU_GET_LIN_DISP(arg)   IMU_CMD(IMU_CMD_GET_LIN_DISP,arg)       /* Arg: Array float[3] for x/y/z */
#define IMU_GET_ANG_VELO(arg)   IMU_CMD(IMU_CMD_GET_ANG_VELO,arg)       /* Arg: Array float[3] for x/y/z */
#define IMU_GET_ANG_DISP(arg)   IMU_CMD(IMU_CMD_GET_ANG_DISP,arg)       /* Arg: Array float[3] for x/y/z */
#define IMU_SET_FILTER_COEF(arg) IMU_CMD(IMU_CMD_SET_FILTER_COEF,arg)   /* Arg: Pointer to float */

/* Writing a specific value to a specific place can cause a software reset 
 * Please refer to NVIC information of ARM Coretex-M7 datasheet
 */
#define SystemReset()   (*((uint32_t*)(0xE000ED0CUL))=(0x05FA0004UL))

/* The thread that responsible for a motor */
typedef struct 
{
    uint32_t id;

    /* File descripters */
    int iMotorFile;
    int iQEFile;
    int iHomeFile;

    /* Motor set-point and its semaphore */
    sem_t theta_sem;
    int32_t theta;  /* unit in absolute steps */
}motor_thread_param_t;

void* motor_thread( void* arg );

motor_thread_param_t motor_param[6];

/* File descriptors */
static int fdIMU = 0;
static int fdHomeSensor = 0;
static int fdTimer = 0;

vector_t current_end_pos[6];
pthread_t xMotorThread[6];
pthread_t xHomeManager;

/****************************************************************************
 * Open all devices and initialize all motor parameters. 
 * If the first attempt failed, the second is performed.
 * If even the second failed, then the function returns error.
 ****************************************************************************/
static void init_motor_param(void)
{
    int retval = OK;
    char buf[60];
    uint32_t i;

    /* Open all devices */
    fdHomeSensor = open( "/dev/home", O_RDWR );
    if( fdHomeSensor <= 0 )
    {
        _warn("Opening HOME sensor failed #1\n");
        fdHomeSensor = open( "/dev/home", O_RDWR );
        DEBUGASSERT ( fdHomeSensor > 0 );
    }

    for( i = 0; i < 6; i++ )
    {
        sprintf( buf, "/dev/motor%d", i );
        motor_param[ i ].iMotorFile = open( buf, O_RDWR );
        if( motor_param[ i ].iMotorFile <= 0 )
        {
            _warn("Opening %s failed #1\n", buf);
            motor_param[ i ].iMotorFile = open( buf, O_RDWR ); /* Attempt another open */
            DEBUGASSERT( motor_param[ i ].iMotorFile > 0 );
        }

        sprintf( buf, "/dev/qe%d", i );
        motor_param[ i ].iQEFile = open( buf, O_RDWR );
        if( motor_param[ i ].iQEFile <= 0 )
        {
            _warn("Opening %s failed #1\n", buf);
            motor_param[ i ].iQEFile = open( buf, O_RDWR ); /* Attempt another open */
            DEBUGASSERT( motor_param[ i ].iQEFile > 0 );
        }

        /* Distribute Home-sensor file handle to all motors */
        motor_param[ i ].iHomeFile = fdHomeSensor;
    }

    fdIMU = open( "/dev/imu", O_RDWR );
    if( fdIMU <= 0 )
    {
        _warn("Opening IMU failed #1\n");
        fdIMU = open( "/dev/imu", O_RDWR );
        DEBUGASSERT( fdIMU > 0 );
    }

    /* Initialze semaphore and its relevant data */
    for( i = 0; i < 6; i++ )
    {
        motor_param[ i ].id = i;
        motor_param[ i ].theta = 0;
        retval = sem_init( &(motor_param[i].theta_sem), 1, 1 );
        if( retval != OK )
        {
            _warn("Failed to initi semaphore %d\n", i);
            DEBUGASSERT( sem_init( &(motor_param[i].theta_sem), 1, 1 ) == OK );
        }
    }
}

#define ROUND_1_1(_x)    ((float)( (int)(_x * 10.0f) ) * 0.1f)
#define ROUND_1(_x)      (roundf(_x * 10.0f) * 0.1f)
#define ROUND_2(_x)      (roundf(_x * 100.0f) * 0.01f)
static void rounding_pos( vector_t* v )
{
    v->x = ROUND_2( v->x );
    v->y = ROUND_2( v->y );
    v->z = ROUND_2( v->z );
}

/****************************************************************************
 * move_robot: Move the platform to the specified position.
 ****************************************************************************/
static void move_robot( vector_t newpos[] )
{
    vector_t t_pos;
    float f;
    int32_t abs_step;

    /* Iterate through all 6 joints */
    for( int i = 0; i < 6; i++ )
    {
        /* Adjust the coordinate to conform with inverse kinematic frame */
        shift_to_motor_frame( &t_pos, &(newpos[i]), i );
        
        /* Convert position into bisep angles. Angle can be + or - */
        f = inverse_kinematic( &(t_pos), ((i%2==0)?true:false) );
        abs_step = (int32_t)(f * ANGLE_TO_STEP_FACTOR);

        /* Send the angle to motor thread */
        DEBUGASSERT( sem_wait( &(motor_param[i].theta_sem) ) == OK );
        motor_param[i].theta = abs_step;
        DEBUGASSERT( sem_post( &(motor_param[i].theta_sem) ) == OK );
    }
}

/*-----------------------------------------------------------------------------------
 * Calculate the new position of each end-effector joints to counter 
 * the current orientation "derived from gravity vector" and position "p"
 *----------------------------------------------------------------------------------*/
void gen_compensate_pos( vector_t newpos[], vector_t currentpos[], vector_t* tilt, vector_t* disp )
{
    float f;
    rot_matrix_t gr;
    float sin_x, cos_x, sin_y, cos_y, sin_z, cos_z;

    /* Generate anti-rotation matrix from tilt */
    sin_x = -sinf( tilt->x );
    cos_x = cosf( tilt->x );
    sin_y = -sinf( tilt->y );
    cos_y = cosf( tilt->y );
    sin_z = -sinf( tilt->z );
    cos_z = cosf( tilt->z );

    /* Tait-Bryan XYZ convension (Look at Wikipedia for more detail) */
    gr[0][0] = cos_y * cos_z;
    gr[0][1] = -( cos_y * sin_z );
    gr[0][2] = sin_y;
    gr[1][0] = ( cos_x * sin_z ) + ( cos_z * sin_x * sin_y );
    gr[1][1] = ( cos_x * cos_z ) - ( sin_x * sin_y * sin_z );
    gr[1][2] = -( cos_y * sin_x );
    gr[2][0] = ( sin_x * sin_z ) - ( cos_x * cos_z * sin_y );
    gr[2][1] = ( cos_z * sin_x ) + ( cos_x * sin_y * sin_z );
    gr[2][2] = cos_x * cos_y;

    /* Initialize result coordinates with initial coordinates then rotate
        those coordinates with respected to the compensation matrix. */
    for( int i = 0; i < 6; i++ )
    {
        /* Compensate angle */
        newpos[i].x = currentpos[i].x;
        newpos[i].y = currentpos[i].y;
        newpos[i].z = currentpos[i].z;
        vector_rotate_m( &(newpos[i]), gr );

        /* Compensate displacement */
        /* If the displacement is zero, then slowly move back to home position */
        if( disp->x > 0.1f || disp->x < -0.1f )
        {
            newpos[i].x -= disp->x;
        }
        else
        {
            f = newpos[i].x - currentpos[i].x;
            if( f > 0.1f || f < -0.1f )
            {
                newpos[i].x -= (f * 0.1f);
            }
        }
        
        if( disp->y > 0.1f || disp->y < -0.1f )
        {
            newpos[i].y -= disp->y;
        }
        else
        {
            f = newpos[i].y - currentpos[i].y;
            if( f > 0.1f || f < -0.1f )
            {
                newpos[i].y -= (f * 0.1f);
            }
        }

        if( disp->z > 0.1f || disp->z < 0.1f )
        {
            newpos[i].z -= disp->z;
        }
        else
        {
            f = newpos[i].z - currentpos[i].z;
            if( f > 0.1f || f < -0.1f )
            {
                newpos[i].z -= (f * 0.1f);
            }
        }

        /* Rounding the decomal points */
        //rounding_pos( &(newpos[i]) );

        _info("Translate pos %d from: %f|%f|%f to %f|%f|%f\n", i,
           currentpos[i].x, currentpos[i].y, currentpos[i].z, 
           newpos[i].x, newpos[i].y, newpos[i].z);
    }
}

/****************************************************************************
 * main_loop
 ****************************************************************************/

static void main_loop(int signo, FAR siginfo_t *siginfo, FAR void *context)
{
    /* This function is meant to run every 40ms (i.e., 25 Hz) */
    float f[4];
    vector_t displacement;
    vector_t tilt;
    vector_t new_pos[6];

    /* Get linear displacement */
    if( IMU_GET_LIN_DISP(f) != OK )
    {
        _err("Failed to get linear displacement from IMU\n");
        return;
    }
    displacement.x = f[0];
    displacement.y = f[1];
    displacement.z = f[2];

    /* Get Tilt */
    if( IMU_GET_TILT(f) != OK )
    {
        _err("Failed to get linear displacement from IMU\n");
        return;
    }
    tilt.x = f[0];
    tilt.y = f[1];
    tilt.z = f[2];

    rounding_pos( &displacement );
    rounding_pos( &tilt );

    _info("Tilt = [%f|%f|%f], D=[%f|%f|%f]\n", tilt.x, tilt.y, tilt.z, displacement.x, displacement.y, displacement.z);
    /* Generate compensated vectors */
    //_info("From data: g = %f|%f|%f and disp = %f|%f|%f\n", gravity.x, gravity.y, gravity.z,
    //displacement.x, displacement.y, displacement.z);
    
    gen_compensate_pos(new_pos, current_end_pos, &tilt, &displacement);

    /* Don't forget to remove this comment */
    move_robot(new_pos);

    /* Update position */
    for(int i = 0;i < 6; i++)
    {
        current_end_pos[i].x = new_pos[i].x;
        current_end_pos[i].y = new_pos[i].y;
        current_end_pos[i].z = new_pos[i].z;
    }
}

/* The thread that continuously check home sensor and reset the corresponding QE */
// void* home_thread(void* arg)
// {
//     uint8_t home_stat;

//     UNUSED(arg);
//     while(1)
//     {
//         if( fdHomeSensor != 0 )
//         {
//             if( read( fdHomeSensor, &home_stat, 1 ) == 1 )
//             {
//                 if( home_stat != 0x3F )
//                 {
//                     /* Some home button trigged (active low) */
//                     if( (home_stat & 0x01) == 0)
//                         QE_RESET( motor_param[0].iQEFile );
//                     if( (home_stat & 0x02) == 0)
//                         QE_RESET( motor_param[1].iQEFile );
//                     if( (home_stat & 0x04) == 0)
//                         QE_RESET( motor_param[2].iQEFile );
//                     if( (home_stat & 0x08) == 0)
//                         QE_RESET( motor_param[3].iQEFile );
//                     if( (home_stat & 0x10) == 0)
//                         QE_RESET( motor_param[4].iQEFile );
//                     if( (home_stat & 0x20) == 0)
//                         QE_RESET( motor_param[5].iQEFile );
//                 }
//             }
//         }
//         usleep(1000);
//     }
// }

/****************************************************************************
 * coordinator_main
 ****************************************************************************/

#if defined(BUILD_MODULE)
int main(int argc, FAR char *argv[])
#else
int coordinator_main(int argc, char *argv[])
#endif
{
    struct timer_notify_s notify;
    struct sigaction act;

    usleep(7000000);    /* Sleep 7 second for all hardware to initialized */

    init_motor_param();

    /* Create threads that manage all motor movement */
    // DEBUGASSERT( pthread_create( &xHomeManager, NULL, home_thread, NULL) == OK );
    for(int i = 0; i < 6; i++)
    {
        DEBUGASSERT( pthread_create( &(xMotorThread[i]), NULL, motor_thread, (void*)(&(motor_param[i])) ) == OK );
    }

    /* Open timer device */
    fdTimer = open("/dev/timer", O_RDONLY);
    if (fdTimer < 0)
    {
      _err( "Cannot open timer device: %d\n", errno);
        usleep(3000000);
        SystemReset();
    }

    /* Set the timer interval */
    while (ioctl(fdTimer, TCIOC_SETTIMEOUT, CONFIG_QUANTUM_STEP) < 0)   /* Set timer to CONFIG_QUANTUM_STEP */
    {
      _err( "ERROR: Failed to set the timer interval: %d\n", errno );
    }

    init_geometry();
    get_init_ee_pos( current_end_pos );

    usleep(100000);

    /* Attach a signal handler to catch the notifications.  NOTE that using
    * signal handler is very slow.  A much more efficient thing to do is to
    * create a separate pthread that waits on sigwaitinfo() for timer events.
    * Much less overhead in that case.
    */
    act.sa_sigaction = main_loop;
    act.sa_flags     = SA_SIGINFO;

    (void)sigfillset(&act.sa_mask);
    (void)sigdelset(&act.sa_mask, CONFIG_COORDINATOR_TIMER_SIGNO);

    while (sigaction(CONFIG_COORDINATOR_TIMER_SIGNO, &act, NULL) != OK)
    {
        _err( "ERROR: Fsigaction failed: %d\n", errno);
    }

    /* Register a callback for notifications using the configured signal.
    *
    * NOTE: If no callback is attached, the timer stop at the first interrupt.
    */

    notify.pid   = getpid();

    notify.event.sigev_notify = SIGEV_SIGNAL;
    notify.event.sigev_signo  = CONFIG_COORDINATOR_TIMER_SIGNO;
    notify.event.sigev_value.sival_ptr = NULL;

    while (ioctl(fdTimer, TCIOC_NOTIFICATION, (unsigned long)((uintptr_t)&notify)) < 0)
    {
        _err("ERROR: Failed to set the timer handler: %d\n", errno);
    }

    /* Start Timer */
    while (ioctl(fdTimer, TCIOC_START, 0) < 0)
    {
        _err( "ERROR: Failed to start the timer: %d\n", errno);
    }

    /* Main Loop, the only infinite loop */
    while(1)
    {
        usleep(1000000UL);
    }
}
