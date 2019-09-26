/****************************************************************************
 * examples/coordinator/coordinator_actor.c
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

#include "coordinator_actor.h"
#include "coordinator_usb.h"

/****************************************************************************
 * Macro
 ****************************************************************************/
#define MOTOR_CMD(id,cmd,arg)   (ioctl(iMotor[id], cmd, (unsigned long)arg))
#define MOTOR_INIT(id)          MOTOR_CMD(id,MOTOR_CMD_INIT,0)
#define MOTOR_RUN(id,param_ptr) MOTOR_CMD(id,MOTOR_CMD_RUN,param_ptr)
#define MOTOR_STOP(id)          MOTOR_CMD(id,MOTOR_CMD_STOP,0)
#define MOTOR_CLR_ALARM(id)     MOTOR_CMD(id,MOTOR_CMD_CLR_ALARM,0)
#define MOTOR_GET_STATE(id,ret) MOTOR_CMD(id,MOTOR_CMD_GET_STATE,ret)
#define MOTOR_GET_CTRL(id,ret)  MOTOR_CMD(id,MOTOR_CMD_GET_STATUS,ret)

#define QE_CMD(id,cmd,arg)      (ioctl(iQEncoder[id], cmd, (unsigned long)arg))
#define QE_GET_VALUE(id,ret)    QE_CMD(id,QEIOC_POSITION,ret)
#define QE_RESET(id)            QE_CMD(id,QEIOC_RESET,0)

#define IMU_CMD(cmd,arg)        (ioctl(iIMU, cmd, (unsigned long)arg))
#define IMU_GET_SAMPLE_RATE(arg) IMU_CMD(IMU_CMD_GET_SAMPLE_RATE,arg)   /* Arg: Pointer to uint32_t */
#define IMU_GET_QUATERNION(arg) IMU_CMD(IMU_CMD_GET_QUATERNION,arg)     /* Arg: Array float[4] */
#define IMU_GET_LIN_ACCEL(arg)  IMU_CMD(IMU_CMD_GET_LIN_ACCEL,arg)      /* Arg: Array float[3] for x/y/z */

/* Writing a specific value to a specific place can cause a software reset 
 * Please refer to NVIC information of ARM Coretex-M7 datasheet
 */
#define SystemReset()   (*((uint32_t*)(0xE000ED0C))=(0x05FA0004))

/****************************************************************************
 * Type definition
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
static char strCommonBuffer[60];
static int iMotor[6] = { 0, 0, 0, 0, 0, 0 };
static int iQEncoder[6] = { 0, 0, 0, 0, 0, 0 };
static int iHomeSensor = 0;
static int iIMU = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static bool IsEmergency(void)
{
    int val;
    for( int i = 0; i < 6; i++ )
    {
        if( iMotor[i] > 0 )
        {
            MOTOR_GET_CTRL(i, &val);
            if( val & 1 ) /* Check emergency status bit */
                return true;
            else
                return false;
        }
    }
    return false;
}

static int SendNAK( char* strToken, char* strParam )
{
    sprintf( strCommonBuffer, "!%sN?%s\n", strToken, strParam );

    return( SendResponse( strCommonBuffer, strlen( strCommonBuffer ) ) );
}

static int SendACK( char* strToken, char* strParam )
{
    sprintf( strCommonBuffer, "!%sA?", strToken );
    if( strParam != NULL )
    {
        strcat( strCommonBuffer, strParam );
    }
    strcat( strCommonBuffer, "\n" );

    return( SendResponse( strCommonBuffer, strlen( strCommonBuffer ) ) );
}

static bool GetUIntParam( char* strKey, uint32_t* u32Out )
{
    char value[12];
    char *t;
    int  i;

    DEBUGASSERT( strKey != NULL );
    DEBUGASSERT( u32Out != NULL );

    /* Extract param as a string from key */
    i = GetParamFromKey( strKey, value, 11 );
    if( ( i <= 0 ) || ( i >= 11) )
    {
        /* Value error */
        return false;
    }

    /* Verify value */
    for( i = 0; ( i < 12 ) && ( value[i] != '\0' ); i++ )
    {
        if( !isdigit( value[i] ) )
            return false;
    }

    /* Convert the string with the standard conversion in stdlib.h */
    *u32Out = strtoul( value, &t, 10 );
    return true;
}

static bool GetIntParam( char* strKey, int32_t* i32Out )
{
    char value[13];
    int  i;

    DEBUGASSERT( strKey != NULL );
    DEBUGASSERT( i32Out != NULL );

    /* Extract param as a string from key */
    i = GetParamFromKey( strKey, value, 12 );
    if( ( i <= 0 ) || ( i >= 12) )
    {
        /* Value error */
        return false;
    }

    /* Verify value */
    if( ( !isdigit( value[0] ) ) && ( value[0] != '-' ) )
        return false;

    for( i = 1; ( i < 13 ) && ( value[i] != '\0' ); i++ )
    {
        if( !isdigit( value[i] ) )
            return false;
    }

    /* Convert the string with the standard conversion in stdlib.h */
    *i32Out = atol( value );
    return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Initialization function */
int InitActor(void)
{
    /* Open all devices */
    for( int i = 0; i < 6; i++ )
    {
        sprintf( strCommonBuffer, "/dev/motor%d", i );
        iMotor[ i ] = open( strCommonBuffer, O_RDWR );
        sprintf( strCommonBuffer, "/dev/qe%d", i );
        iQEncoder[ i ] = open( strCommonBuffer, O_RDWR );
    }

    iHomeSensor = open( "/dev/home", O_RDWR );
    iIMU = open( "/dev/imu", O_RDWR );

    return OK;
}

/****************************************************************************
 * Actor functions for each command. It may utilize some local functions.
 ****************************************************************************/

int DoUnknown( char* strToken )
{
    _err( "Received unknown command %s\n", strToken );
    return( SendNAK( strToken, "v=CMD Unknown" ) );
}

int DoReboot( char* strToken )
{
    UNUSED( strToken );

    _err( "System is resetting\n" );
    SystemReset();
    return OK;
}

int DoGetAvailableMotor( char* strToken )
{
    int i;
    char tmp[2];

    _info( "Requesting for available motor\n" );
    sprintf( strCommonBuffer, "!%sA?m=", strToken );
    tmp[1] = 0;

    /* List only available motor device file */
    for( i = 0; i < 6; i++ )
    {
        if( iMotor[ i ] > 0 )
        {
            tmp[ 0 ] = (char)i + '0';
            strcat( strCommonBuffer, tmp );
        }
    }

    /* Closing the response */
    strcat( strCommonBuffer, "\n" );

    return( SendResponse( strCommonBuffer, strlen( strCommonBuffer ) ) );
}

int DoInitMotor( char* strToken )
{
    uint32_t    motor;

    _info( "Initialize motor\n" );
    if( !GetUIntParam( "m", &motor ) ) 
    {
        return( SendNAK( strToken, "v=Motor Unspecified" ) );
    }

    if( motor >= 6 )
    {
        return( SendNAK( strToken, "v=Unknown Motor" ) );
    }

    if( iMotor[motor] <= 0 )
    {
        return( SendNAK( strToken, "v=Unreachable Motor" ) );
    }

    if( MOTOR_INIT( motor ) != OK )
        return( SendNAK( strToken, "v=Init Failed" ) );
    else
        return( SendACK( strToken, NULL ) );
}

int DoGetMotorStatus( char* strToken )
{
    uint32_t    motor;
    uint32_t    state, ctrl_status;

    _info( "Requesting for motor status\n" );
    if( !GetUIntParam( "m", &motor ) ) 
    {
        return( SendNAK( strToken, "v=Motor Unspecified" ) );
    }

    if( motor >= 6 )
    {
        return( SendNAK( strToken, "v=Unknown Motor" ) );
    }

    if( iMotor[motor] <= 0 )
    {
        return( SendNAK( strToken, "v=Unreachable Motor" ) );
    }

    /* Get motor info */
    if( MOTOR_GET_STATE( motor, &state ) != OK )
    {
        return( SendNAK( strToken, "v=Unable to Get State" ) );
    }
    if( MOTOR_GET_CTRL( motor, &ctrl_status ) != OK )
    {
        return( SendNAK( strToken, "v=Unable to Get Control Status" ) );
    }

    /* Assemble response packet */
    /* Header */
    sprintf( strCommonBuffer, "!%sA?", strToken );

    /* State */
    if( state <  0 )
    {
        strcat( strCommonBuffer, "s=ERROR;" );
    }
    else
    {
        switch( state )
        {
            case MOTOR_UNINIT:
                strcat( strCommonBuffer, "s=UNITIT;" );
                break;
            case MOTOR_READY:
                strcat( strCommonBuffer, "s=READY;" );
                break;
            case MOTOR_ALARM:
                strcat( strCommonBuffer, "s=ALARM;" );
                break;
            case MOTOR_UNAVAILABLE:
                strcat( strCommonBuffer, "s=UNAVAIL;" );
                break;
            default:
                strcat( strCommonBuffer, "s=UNKNOWN;" );
                break;
        }
    }

    /* Control status */
    strcat( strCommonBuffer, "ctrl=" );
    if( ctrl_status & 0x100 )   /* Complete */
    {
        strcat( strCommonBuffer, "C" );
    }
    else
    {
        strcat( strCommonBuffer, "-" );
    }
    if( ctrl_status & 0x10 )    /* Alarm */
    {
        strcat( strCommonBuffer, "A" );
    }
    else
    {
        strcat( strCommonBuffer, "-" );
    }
    if( ctrl_status & 0x1 )     /* Emergency */
    {
        strcat( strCommonBuffer, "E\n" );
    }
    else
    {
        strcat( strCommonBuffer, "-\n" );
    }

    return( SendResponse( strCommonBuffer, strlen( strCommonBuffer ) ) );
}

int DoClearMotorAlarm( char* strToken )
{
    uint32_t    motor;
    int         state;

    _info( "Clear motor alarm\n" );
    if( !GetUIntParam( "m", &motor ) ) 
    {
        return( SendNAK( strToken, "v=Motor Unspecified" ) );
    }

    if( motor >= 6 )
    {
        return( SendNAK( strToken, "v=Unknown Motor" ) );
    }

    if( iMotor[motor] <= 0 )
    {
        return( SendNAK( strToken, "v=Unreachable Motor" ) );
    }

    /* Get current motor state */
    MOTOR_GET_STATE( motor, &state );
    if( state == MOTOR_ALARM )
    {
        if( MOTOR_CLR_ALARM( motor ) < 0 )
        {
            return( SendNAK ( strToken, "v=Alarm persist" ) );
        }
        else
        {
            return( SendACK( strToken, NULL ) );
        }
    }
    else
    {
        return( SendNAK( strToken, "v=Motor not alarm" ) );
    }
}

int DoStopMotor( char* strToken )
{
    uint32_t    motor;

    _info( "Stop motor\n" );
    if( !GetUIntParam( "m", &motor ) ) 
    {
        return( SendNAK( strToken, "v=Motor Unspecified" ) );
    }

     if( motor >= 6 )
    {
        return( SendNAK( strToken, "v=Unknown Motor" ) );
    }

   if( iMotor[motor] <= 0 )
    {
        return( SendNAK( strToken, "v=Unreachable Motor" ) );
    }

    MOTOR_STOP( motor );

    return( SendACK( strToken, NULL ) );
}

int DoRunMotor( char* strToken )
{
    uint32_t                    motor;
    int                         state;
    uint32_t                    speed, step;
    char                        dir[4];
    struct motor_run_param_t    m_param;

    _info( "Run motor\n" );
    if( !GetUIntParam( "m", &motor ) ) 
    {
        return( SendNAK( strToken, "v=Motor Unspecified" ) );
    }

    if( motor >= 6 )
    {
        return( SendNAK( strToken, "v=Unknown Motor" ) );
    }

    if( iMotor[motor] <= 0 )
    {
        return( SendNAK( strToken, "v=Unreachable Motor" ) );
    }

    /* Verify the parameters */
    if( !GetUIntParam( "spd", &speed ) )
    {
        return( SendNAK( strToken, "v=Cannot Find a Valid Speed" ) );
    }

    if( speed >= 65535 )
    {
        return( SendNAK( strToken, "v=Requested Speed Exceeds Max" ) );
    }

    if( !GetUIntParam( "stp", &step ) )
    {
        return( SendNAK( strToken, "v=Cannot Find a Valid Step" ) );
    }

    if( step >= 65535 )
    {
        return( SendNAK( strToken, "v=Requested Step Exceeds Max" ) );
    }

    if( GetParamFromKey( "dir", dir, 3 ) <= 0 )
    {
        return( SendNAK( strToken, "v=Invalid Direction" ) );
    }

    if( strcmp( dir, "cw" ) == 0 )
    {
        m_param.is_cw = true;
    }
    else
    {
        if( strcmp( dir, "ccw" ) == 0 )
        {
            m_param.is_cw = false;
        }
        else
        {
            return( SendNAK( strToken, "v=Invalid Direction" ) );
        }
    }

    m_param.speed = (uint16_t)speed;
    m_param.step = (uint16_t)step;

    /* Check wheter motor is ready */
    MOTOR_GET_STATE( motor, &state );

    if( state != MOTOR_READY )
    {
        return( SendNAK( strToken, "v=Motor Not Ready" ) );
    }

    /* Run the motor */
    if( MOTOR_RUN( motor, &m_param ) != OK )
    {
        return( SendNAK( strToken, "v=RUN Command Failed" ) );
    }
    else
    {
        return( SendACK( strToken, NULL ) );
    }
}

int DoGetQEValue( char* strToken )
{
    uint32_t    qe;
    int32_t     val;

    _info( "Read encoder value\n" );
    if( !GetUIntParam( "qe", &qe ) ) 
    {
        return( SendNAK( strToken, "v=Encoder Unspecified" ) );
    }

    if( qe >= 6 )
    {
        return( SendNAK( strToken, "v=Unknown Encoder" ) );
    }

    if( iQEncoder[qe] <= 0 )
    {
        return( SendNAK( strToken, "v=Unreachable Encoder" ) );
    }

    if( QE_GET_VALUE( qe, &val ) != OK )
    {
        return( SendNAK( strToken, "v=Unable to get encoder value" ) );
    }

    sprintf( strCommonBuffer, "!%sA?v=%ld\n", strToken, val );
    return( SendResponse( strCommonBuffer, strlen( strCommonBuffer ) ) );    
}

int DoResQEValue( char* strToken )
{
    uint32_t    qe;

    _info( "Reset encoder value\n" );
    if( !GetUIntParam( "qe", &qe ) ) 
    {
        return( SendNAK( strToken, "v=Encoder Unspecified" ) );
    }

    if( qe >= 6 )
    {
        return( SendNAK( strToken, "v=Unknown Encoder" ) );
    }

    if( iQEncoder[qe] <= 0 )
    {
        return( SendNAK( strToken, "v=Unreachable Encoder" ) );
    }

    if( QE_RESET( qe ) != OK )
    {
        return( SendNAK( strToken, "v=Failed to reset encoder" ) );
    }
    else
    {
         return( SendACK( strToken, NULL ) );
    }

}

int DoHomeValue( char* strToken )
{
    uint8_t val;

    _info( "Read home sensor\n" );
    if( iHomeSensor <= 0 )
    {
        return( SendNAK( strToken, "v=Home Sensor Lost" ) );
    }

    read( iHomeSensor, &val, 1 );

    sprintf( strCommonBuffer, "!%sA?v=%02X\n", strToken, val );
    return( SendResponse( strCommonBuffer, strlen( strCommonBuffer ) ) );    
}

int DoGetIMUSampleRate( char* strToken )
{   
    uint32_t rate;

    if( iIMU <= 0 )
    {
        return( SendNAK( strToken, "v=No IMU Found" ) );
    }

    if( IMU_GET_SAMPLE_RATE( &rate ) != OK )
    {
        return( SendNAK( strToken, "v=Fail to Get IMU Sample Rate" ) );
    }

    sprintf( strCommonBuffer, "!%sA?v=%d\n", strToken, rate );
    return( SendResponse( strCommonBuffer, strlen( strCommonBuffer ) ) );    
}

int DoGetIMUQuaternion( char* strToken )
{   
    float q[4];

    if( iIMU <= 0 )
    {
        return( SendNAK( strToken, "v=No IMU Found" ) );
    }

    if( IMU_GET_QUATERNION( q ) != OK )
    {
        return( SendNAK( strToken, "v=Fail to Get IMU Quaternion" ) );
    }

    sprintf( strCommonBuffer, "!%sA?v=%f-%f-%f-%f\n", strToken, q[0], q[1], q[2], q[3] );
    return( SendResponse( strCommonBuffer, strlen( strCommonBuffer ) ) );    
}

int DoGetIMULinAccel( char* strToken )
{   
    float a[3];

    if( iIMU <= 0 )
    {
        return( SendNAK( strToken, "v=No IMU Found" ) );
    }

    if( IMU_GET_LIN_ACCEL( a ) != OK )
    {
        return( SendNAK( strToken, "v=Fail to Get IMU Linear Accelteration" ) );
    }

    sprintf( strCommonBuffer, "!%sA?v=%f-%f-%f\n", strToken, a[0], a[1], a[2] );
    return( SendResponse( strCommonBuffer, strlen( strCommonBuffer ) ) );    
}
