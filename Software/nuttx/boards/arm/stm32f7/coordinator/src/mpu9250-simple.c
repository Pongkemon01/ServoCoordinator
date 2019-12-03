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
#include <nuttx/wqueue.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <math.h>

#include <sys/types.h>
#include <nuttx/i2c/i2c_master.h>
#include <arch/board/board.h>
#include <arch/board/imu.h>

#include "stm32_i2c.h"

#ifndef CONFIG_I2C
# error "IMU requires i2c driver"
#endif
#ifndef CONFIG_SCHED_HPWORK
# error "IMU requires a high-priority work queue"
#endif
#ifndef CONFIG_CLOCK_MONOTONIC
# error "IMU requires CONFIG_CLOCK_MONOTONIC"
#endif
/************************************************************************************
 * Private Data
 ************************************************************************************/
#define MPU_WORK_QUEUE          HPWORK

#define I2C_SPEED               250000  /* I2C Clock speed */

#define MPU9250_ADDRESS         (0x68)  /* AD0 = 0 */
#define AK8963_ADDRESS          (0x0C)  /* Address for direct access */

#define SAMPLE_RATE             100UL     /* Sensor sample rate (Hz.) max = 1000 */

#define SAMPLE_PERIOD           (1000000UL/(CONFIG_USEC_PER_TICK * SAMPLE_RATE))

/* See also MPU-9250 Register Map and Descriptions, Revision 4.0, RM-MPU-9250A-00 */
/* Magnetometer Registers */
#define AK8963_WHO_AM_I         0x00    /* should return 0x48 */
#define AK8963_INFO             0x01
#define AK8963_ST1              0x02    /* data ready status bit 0 */
#define AK8963_XOUT_L           0x03    /* data */
#define AK8963_XOUT_H           0x04
#define AK8963_YOUT_L           0x05
#define AK8963_YOUT_H           0x06
#define AK8963_ZOUT_L           0x07
#define AK8963_ZOUT_H           0x08
#define AK8963_ST2              0x09  /* Data overflow bit 3 and data read error status bit 2 */
#define AK8963_CNTL             0x0A  /* Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0 */
#define AK8963_ASTC             0x0C  /* Self test control */
#define AK8963_I2CDIS           0x0F  /* I2C disable */
#define AK8963_ASAX             0x10  /* Fuse ROM x-axis sensitivity adjustment value */
#define AK8963_ASAY             0x11  /* Fuse ROM y-axis sensitivity adjustment value */
#define AK8963_ASAZ             0x12  /* Fuse ROM z-axis sensitivity adjustment value */

/* Gyro and accelero Registers */
#define SELF_TEST_X_GYRO        0x00                  
#define SELF_TEST_Y_GYRO        0x01                                                                          
#define SELF_TEST_Z_GYRO        0x02

#define SELF_TEST_X_ACCEL       0x0D
#define SELF_TEST_Y_ACCEL       0x0E    
#define SELF_TEST_Z_ACCEL       0x0F

#define SELF_TEST_A             0x10

#define XG_OFFSET_H             0x13  /* User-defined trim values for gyroscope */
#define XG_OFFSET_L             0x14
#define YG_OFFSET_H             0x15
#define YG_OFFSET_L             0x16
#define ZG_OFFSET_H             0x17
#define ZG_OFFSET_L             0x18
#define SMPLRT_DIV              0x19
#define CONFIG                  0x1A
#define GYRO_CONFIG             0x1B
#define ACCEL_CONFIG            0x1C
#define ACCEL_CONFIG2           0x1D
#define LP_ACCEL_ODR            0x1E   
#define WOM_THR                 0x1F   

#define MOT_DUR                 0x20  /* Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms */
#define ZMOT_THR                0x21  /* Zero-motion detection threshold bits [7:0] */
#define ZRMOT_DUR               0x22  /* Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms */

#define FIFO_EN                 0x23
#define I2C_MST_CTRL            0x24   
#define I2C_SLV0_ADDR           0x25
#define I2C_SLV0_REG            0x26
#define I2C_SLV0_CTRL           0x27
#define I2C_SLV1_ADDR           0x28
#define I2C_SLV1_REG            0x29
#define I2C_SLV1_CTRL           0x2A
#define I2C_SLV2_ADDR           0x2B
#define I2C_SLV2_REG            0x2C
#define I2C_SLV2_CTRL           0x2D
#define I2C_SLV3_ADDR           0x2E
#define I2C_SLV3_REG            0x2F
#define I2C_SLV3_CTRL           0x30
#define I2C_SLV4_ADDR           0x31
#define I2C_SLV4_REG            0x32
#define I2C_SLV4_DO             0x33
#define I2C_SLV4_CTRL           0x34
#define I2C_SLV4_DI             0x35
#define I2C_MST_STATUS          0x36
#define INT_PIN_CFG             0x37
#define INT_ENABLE              0x38
#define DMP_INT_STATUS          0x39  /* Check DMP interrupt */
#define INT_STATUS              0x3A
#define ACCEL_XOUT_H            0x3B
#define ACCEL_XOUT_L            0x3C
#define ACCEL_YOUT_H            0x3D
#define ACCEL_YOUT_L            0x3E
#define ACCEL_ZOUT_H            0x3F
#define ACCEL_ZOUT_L            0x40
#define TEMP_OUT_H              0x41
#define TEMP_OUT_L              0x42
#define GYRO_XOUT_H             0x43
#define GYRO_XOUT_L             0x44
#define GYRO_YOUT_H             0x45
#define GYRO_YOUT_L             0x46
#define GYRO_ZOUT_H             0x47
#define GYRO_ZOUT_L             0x48
#define EXT_SENS_DATA_00        0x49
#define EXT_SENS_DATA_01        0x4A
#define EXT_SENS_DATA_02        0x4B
#define EXT_SENS_DATA_03        0x4C
#define EXT_SENS_DATA_04        0x4D
#define EXT_SENS_DATA_05        0x4E
#define EXT_SENS_DATA_06        0x4F
#define EXT_SENS_DATA_07        0x50
#define EXT_SENS_DATA_08        0x51
#define EXT_SENS_DATA_09        0x52
#define EXT_SENS_DATA_10        0x53
#define EXT_SENS_DATA_11        0x54
#define EXT_SENS_DATA_12        0x55
#define EXT_SENS_DATA_13        0x56
#define EXT_SENS_DATA_14        0x57
#define EXT_SENS_DATA_15        0x58
#define EXT_SENS_DATA_16        0x59
#define EXT_SENS_DATA_17        0x5A
#define EXT_SENS_DATA_18        0x5B
#define EXT_SENS_DATA_19        0x5C
#define EXT_SENS_DATA_20        0x5D
#define EXT_SENS_DATA_21        0x5E
#define EXT_SENS_DATA_22        0x5F
#define EXT_SENS_DATA_23        0x60
#define MOT_DETECT_STATUS       0x61
#define I2C_SLV0_DO             0x63
#define I2C_SLV1_DO             0x64
#define I2C_SLV2_DO             0x65
#define I2C_SLV3_DO             0x66
#define I2C_MST_DELAY_CTRL      0x67
#define SIGNAL_PATH_RESET       0x68
#define MOT_DETECT_CTRL         0x69
#define USER_CTRL               0x6A    /* Bit 7 enable DMP, bit 3 reset DMP */
#define PWR_MGMT_1              0x6B    /* Device defaults to the SLEEP mode */
#define PWR_MGMT_2              0x6C

#define DMP_BANK                0x6D  /* Activates a specific bank in the DMP */
#define DMP_ADDRESS             0x6E  /* Set read/write pointer to a specific start address in specified DMP bank */
#define DMP_REG                 0x6F  /* Register in DMP from which to read or to which to write */
#define DMP_FW_START_H          0x70
#define DMP_FW_START_L          0x71

#define FIFO_COUNTH             0x72
#define FIFO_COUNTL             0x73
#define FIFO_R_W                0x74
#define WHO_AM_I_MPU9250        0x75 /* Should return 0x71 */
#define XA_OFFSET_H             0x77
#define XA_OFFSET_L             0x78
#define YA_OFFSET_H             0x7A
#define YA_OFFSET_L             0x7B
#define ZA_OFFSET_H             0x7D
#define ZA_OFFSET_L             0x7E

// Set initial input parameters
enum AK8963_Mode_e
{
    AK8963_MODE_PWRDN = 0,      /* Power down */
    AK8963_MODE_SINGLE = 1,     /* Single measurement */
    AK8963_MODE_CON8HZ = 2,     /* 8Hz continuous measurement */
    AK8963_MODE_CON100HZ = 6,   /* 100Hz continuous measurement */
    AK8963_MODE_EXTTRIG = 4,    /* External trigger measurement */
    AK8963_MODE_SELFTEST = 8,   /* Self-test */
    AK8963_MODE_ROMACC = 0xF,   /* Fuse ROM access */
};

enum AK8963_Resolution_e {
    MFS_14BITS = 0,             /* 0.6 mG per LSB */
    MFS_16BITS = 0x10,          /* 0.15 mG per LSB */
};

enum Accel_Scale_e {
    AFS_MASK = 0x18,
    AFS_2G = 0,
    AFS_4G = 8,
    AFS_8G = 0x10,
    AFS_16G = 0x18,
};

enum Gyro_Scale_e {
    GFS_MASK = 0x18,
    GFS_250DPS = 0,
    GFS_500DPS = 8,
    GFS_1000DPS = 0x10,
    GFS_2000DPS = 0x18,
};

#define G_SCALE     GFS_250DPS
#define A_SCALE     AFS_2G
#define M_RES       MFS_16BITS

/* The follwing factor must be set according to the scales and resolution above */
#define G_FACTOR    ( 250.0f / 32768.0f )
#define A_FACTOR    ( 2.0f / 32768.0f )
#define M_FACTOR    ( 10.0f * 4912.0f / 32760.0f )

#define F_PI        3.1415926535897932384626433832795029f
#define EARTH_G     9.8f
#define DEG_TO_RAD  ( F_PI / 180.0f )
#define RAD_TO_DEG  ( 180.0f / F_PI )

#define ROUND_1(_x)      ((float)( (int)(_x * 10.0f + 0.5f) ) * 0.1f)
#define ROUND_2(_x)      ((float)( (int)(_x * 100.0f + 0.5f) ) * 0.01f)

#define EMA_A   0.3f

typedef struct
{
    float x, y, z;
}vector_t;

/* This is the bias value of the current MPU9250. These values must be re-adjusted if sensor is changed */
static int16_t ci16ReadBias[7] = {
    674, 323, -665, 0, -341, 322, 123
};

static const struct i2c_config_s   mpu_i2c_config =
{
    .frequency  = I2C_SPEED,
    .address    = MPU9250_ADDRESS,
    .addrlen    = 7                    /* 7-bit address */
};
static struct i2c_master_s*  i2c_bus = NULL;

static uint32_t lastupdate = 0, tick_count = 0;
static float delta_s;    /* Time period used for integration */

#define INTEGRATION_WAIT_PERIOD  15  /* The amount of second waiting for the integration function to activate */
static unsigned int integration_enabled = 0;
static vector_t gravity; /* Gravity vector */
static vector_t accel; /* Acceleration (acceleration with gravity component subtracted) */
static vector_t velo;  /* Velocity gotten from integrate the acel */
static vector_t disp;  /* Diaplacement gotten from integrate the velo */
static vector_t a_state, v_state, d_state;  /* State for high-pass filter */
static vector_t omega, theta;  /* Angular velocity and angular displacement */
static vector_t o_state, t_state;   /* State for high-pass filter */

/* Data for complement filter to fust gyro with accel to get gravity angle */
#define COMP_FILTER_BIAS 0.98
static vector_t xTiltAngle =
{
    .x = 0,
    .y = 0,
    .z = 0
};

/* Data for 5-element median filter for accelerometer. These arrays
 * must be initialized with values greater than 0 to prevent divided-by-zero
 * of the further calculation of the filter result */
static float t_x[5] = {0.0001f, 0.0001f, 0.0001f, 0.0001f, 0.0001f};
static float t_y[5] = {0.0001f, 0.0001f, 0.0001f, 0.0001f, 0.0001f};
static float t_z[5] = {0.0001f, 0.0001f, 0.0001f, 0.0001f, 0.0001f};

static float fEmaCoefficient = EMA_A;

static struct work_s worker;       /* kernel worker queue data */
static bool bMPUReady;              /* Indicate whether MPU is ready */

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

/*-----------------------------------------------------------------------------------
 * 5-element median filter for acceleration
 *----------------------------------------------------------------------------------*/
static float get_median( float arr[] )
{
    uint32_t ind[5], t;
    int i, j;

    /* Init index as linear */
    ind[0] = 0;
    ind[1] = 1;
    ind[2] = 2;
    ind[3] = 3;
    ind[4] = 4;

    /* Sort */
    for( i = 5; i > 0; i-- )
    {
        for( j = 1; j < i; j++ )
        {
            if(arr[ind[j - 1]] > arr[ind[j]] )
            {
                t = ind[j - 1];
                ind[j - 1] = ind[j];
                ind[j] = t;
            }
        }
    }

    /* Return the middle */
    return( arr[ind[2]] );
}

static void median_filter( vector_t* out_accel, float ax, float ay, float az )
{
    /* Shifting */
    for( int i = 1; i < 5; i++ )
    {
        t_x[i - 1] = t_x[i];
        t_y[i - 1] = t_y[i];
        t_z[i - 1] = t_z[i];
    }

    /* Add data  */
    t_x[4] = ax;
    t_y[4] = ay;
    t_z[4] = az;

    out_accel->x = get_median( t_x );
    out_accel->y = get_median( t_y );
    out_accel->z = get_median( t_z );
}

/*-----------------------------------------------------------------------------------
 * Helper function : Fast inverse square-root ( i.e., 1/sqrt(x) )
 * See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
 *----------------------------------------------------------------------------------*/
static float InvSqrt( float x )
{
    float halfx = 0.5f * x;
    union {
        float f;
        uint32_t i;
    } conv = {x};       /* evil floating point bit level hacking */

    conv.i = 0x5f3759df - ( conv.i >> 1 ); /* 0x5f3759df is just a magic number */

    /* The next line can be repeated any number of times to increase accuracy */
    conv.f *= ( 1.5f - ( halfx * conv.f * conv.f ) );
    conv.f *= ( 1.5f - ( halfx * conv.f * conv.f ) ); /* 2nd iteration to improve accuracy */
    return conv.f;
}

#define FILTER_TYPE 2   /* Type of complementary filter to use. 1 or 2 or.... */

#if (FILTER_TYPE == 1)
/* Another implementation of complementary filter from:
 * http://scolton.blogspot.com/2012/09/fun-with-complementary-filter-multiwii.html
 * which based on the implementation #3 in:
 * http://scolton.blogspot.com/2012/08/kk20-firmware-mod-unleash-complementary.html
 * The pseudo code of the implementation #3 for an angle is:
 *
 *      filtered_angle = filtered_angle + gyro_rate*dt;
 *      error_angle = acc_angle - filtered_angle;
 *      filtered_angle = filtered_angle + (1-A)*error_angle;
 *
 * where A is the COMP_FILTER_BIAS.

 */
static void complement_filter( float ax, float ay, float az, float gx, float gy, float gz )
{
    float err_angle;

    xTiltAngle.x = xTiltAngle.x + (gx * delta_s);
    err_angle = atanf( ay * InvSqrt( (ax*ax) + (az*az) ) ) - xTiltAngle.x;
    xTiltAngle.x = xTiltAngle.x + ( (1.0f - COMP_FILTER_BIAS) * err_angle );

    xTiltAngle.y = xTiltAngle.y + (gy * delta_s);
    err_angle = atanf( -ax * InvSqrt( (ay*ay) + (az*az) ) ) - xTiltAngle.y;
    xTiltAngle.y = xTiltAngle.y + ( (1.0f - COMP_FILTER_BIAS) * err_angle );

    xTiltAngle.z = COMP_FILTER_BIAS * ( xTiltAngle.z + (gz * delta_s) ); /* Take only gyro for yaw */
}
#endif

#if (FILTER_TYPE == 2)
/* Complementary filter that fuses Gyro with Accel sensors to get the tilt angle */
static void complement_filter( float ax, float ay, float az, float gx, float gy, float gz )
{
   xTiltAngle.x = ( COMP_FILTER_BIAS * ( xTiltAngle.x + (gx * delta_s) ) ) + ( (1.0f - COMP_FILTER_BIAS) * atanf( ay * InvSqrt( (ax*ax) + (az*az) ) ) );
   xTiltAngle.y = ( COMP_FILTER_BIAS * ( xTiltAngle.y + (gy * delta_s) ) ) + ( (1.0f - COMP_FILTER_BIAS) * atanf( -ax * InvSqrt( (ay*ay) + (az*az) ) ) );
   xTiltAngle.z = COMP_FILTER_BIAS * ( xTiltAngle.z + (gz * delta_s) ); /* Take only gyro for yaw */
}
#endif

/************************************************************************************
 * Funtion : MPUWriteBytes and COMWriteBytes
 * Write bytes of data to MPU (address 0x68) or Compass (address 0x0C)
 ************************************************************************************/
static int MPUWrite1Byte( uint8_t addr, uint8_t data )
{
    uint8_t buffer[2];
    buffer[0] = addr;
    buffer[1] = data;
    return( i2c_write( i2c_bus, &mpu_i2c_config, buffer, 2 ) );
}

/************************************************************************************
 * Funtion : MPUReadByte and COMReadBytes
 * Read bytes of data from MPU (address 0x68) or Compass (address 0x0C)
 ************************************************************************************/
static inline int MPUReadBytes( uint8_t addr, uint8_t* data, uint8_t len )
{
    return( i2c_writeread( i2c_bus, &mpu_i2c_config, &addr, 1, data, len ) );
}

/************************************************************************************
 * Funtions to read various sensor output
 ************************************************************************************/
static int ReadMPU9250Data( int16_t* destination )
{
    uint8_t rawData[14];  // x/y/z accel register data stored here
    int     retval;

    // Read the 14 raw data registers into data array
    retval = MPUReadBytes( ACCEL_XOUT_H, rawData, 14 );  
    if( retval != OK )
        return retval;

    // Turn the MSB and LSB into a signed 16-bit value, then remove bias
    destination[0] = (int16_t)(((uint16_t)rawData[0] << 8U) | ((uint16_t)rawData[1])) - ci16ReadBias[0] ;
    destination[1] = (int16_t)(((uint16_t)rawData[2] << 8U) | ((uint16_t)rawData[3])) - ci16ReadBias[1] ;  
    destination[2] = (int16_t)(((uint16_t)rawData[4] << 8U) | ((uint16_t)rawData[5])) - ci16ReadBias[2] ; 
    destination[3] = (int16_t)(((uint16_t)rawData[6] << 8U) | ((uint16_t)rawData[7])) - ci16ReadBias[3] ;   
    destination[4] = (int16_t)(((uint16_t)rawData[8] << 8U) | ((uint16_t)rawData[9])) - ci16ReadBias[4] ;  
    destination[5] = (int16_t)(((uint16_t)rawData[10] << 8U) | ((uint16_t)rawData[11])) - ci16ReadBias[5] ;  
    destination[6] = (int16_t)(((uint16_t)rawData[12] << 8U) | ((uint16_t)rawData[13])) - ci16ReadBias[6] ; 

    return OK;
}


/************************************************************************************
 * Funtion : InitMPU9250
 * Initialize devices.
 ************************************************************************************/
static int InitMPU9250(void)
{
    int     retval;

    /* wake up device */
    retval = MPUWrite1Byte( PWR_MGMT_1, 0 ); /* Clear sleep mode bit (6), enable all sensors */
    if( retval != OK )
        return retval;
    usleep(10000); /* Wait for all registers to reset */

    /* get stable time source */
    retval = MPUWrite1Byte( PWR_MGMT_1, 1 ); /* Auto select clock source to be PLL */
    if( retval != OK )
        return retval;
    usleep(10000); /* Wait for all registers to reset */
  
    /* Configure Gyro, Thermometer, and accelerometer to 1 kHz sampling rate
    * Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, 
    * respectively; minimum delay time for this setting is 5.9 ms, which means
    * sensor fusion update rates cannot be higher than 1 / 0.0059 = 170 Hz
    * DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    */
    retval = MPUWrite1Byte( CONFIG, 0x03 );
    if( retval != OK )
        return retval;
 
    /* Set gyroscope full scale range. Range selects FS_SEL and GFS_SEL are 0 - 3 */
    retval = MPUWrite1Byte( GYRO_CONFIG, G_SCALE );
    if( retval != OK )
        return retval;

    /* Set accelerometer full-scale range configuration */
    retval = MPUWrite1Byte( ACCEL_CONFIG, A_SCALE );
    if( retval != OK )
        return retval;

    /* Set accelerometer sample rate configuration to 1 kHz with bandwidth of 44.8 Hz */
    retval = MPUWrite1Byte( ACCEL_CONFIG2, 0x03 );
    if( retval != OK )
        return retval;

    /* Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV) */
    retval = MPUWrite1Byte( SMPLRT_DIV, ( 1000 / SAMPLE_RATE ) - 1 );
    if( retval != OK )
        return retval;
  
    /* The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
     * but all these rates are further reduced further by the SMPLRT_DIV setting to
     * match with the SAMPLE_RATE */

    /* Enable I2C bypass mode by setting I2C_BYPASS_EN so additional chips 
     * can join the I2C bus and all can be controlled directly.
     */
    return( MPUWrite1Byte( INT_PIN_CFG, 0x02 ) ); /* Cuurently we don't have interrupt */
}

/************************************************************************************
 * Accelerometer and gyroscope self test; check calibration with respect to
 * factory settings. It returns percent deviation from factory trim values.
 * +/- 14 or less deviation is a pass
 ************************************************************************************/
static void MPUSelfTest( float * deviation ) /* deviation must be 6-element array */
{
    uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
    uint8_t selfTest[6];
    int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
    float factoryTrim[6];
    uint8_t FS = 0;
    int     ii;
   
    MPUWrite1Byte( SMPLRT_DIV, 0x00 );    // Set gyro sample rate to 1 kHz
    MPUWrite1Byte( CONFIG, 0x02 );        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
    MPUWrite1Byte( GYRO_CONFIG, FS<<3U );  // Set full scale range for the gyro to 250 dps
    MPUWrite1Byte( ACCEL_CONFIG2, 0x02 ); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
    MPUWrite1Byte( ACCEL_CONFIG, FS<<3U ); // Set full scale range for the accelerometer to 2 g

    for( ii = 0; ii < 200; ii++ ) 
    { 
        /* get average current values of gyro and acclerometer */
  
        MPUReadBytes( ACCEL_XOUT_H, rawData, 6 );        // Read the six raw data registers into data array
        aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
        aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
        aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  
        MPUReadBytes( GYRO_XOUT_H, rawData, 6 );       // Read the six raw data registers sequentially into data array
        gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
        gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
        gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
    }
  
    for ( ii =0; ii < 3; ii++ ) 
    {  /* Get average of 200 values and store as average current readings */
        aAvg[ii] /= 200;
        gAvg[ii] /= 200;
    }

    /* Copy the average value as the bias value */
    /* Gyro */
    ci16ReadBias[4] = gAvg[0];
    ci16ReadBias[5] = gAvg[1];
    ci16ReadBias[6] = gAvg[2];
    /* We cannot do the same for accelerometer because we need to preserve gravity bias.
     * Therefore, we stick with static pre-defined bias values
     */

    /* Configure the accelerometer for self-test */
    MPUWrite1Byte( ACCEL_CONFIG, 0xE0 ); // Enable self test on all three axes and set accelerometer range to +/- 2 g
    MPUWrite1Byte( GYRO_CONFIG,  0xE0 ); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
    usleep(25000);  // Delay a while to let the device stabilize

    for( ii = 0; ii < 200; ii++) 
    { 
        /* get average self-test values of gyro and acclerometer */
  
        MPUReadBytes( ACCEL_XOUT_H, rawData, 6 );        // Read the six raw data registers into data array
        aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
        aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
        aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  
        MPUReadBytes( GYRO_XOUT_H, rawData, 6 );       // Read the six raw data registers sequentially into data array
        gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
        gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
        gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
    }
  
    for ( ii =0; ii < 3; ii++ ) 
    {  /* Get average of 200 values and store as average self-test readings */
        aSTAvg[ii] /= 200;
        gSTAvg[ii] /= 200;
    }   
  
    /* Configure the gyro and accelerometer for normal operation */
    MPUWrite1Byte( ACCEL_CONFIG, 0x00 );  
    MPUWrite1Byte( GYRO_CONFIG,  0x00 );  
    usleep(25000);  // Delay a while to let the device stabilize
   
    /* Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg */
    MPUReadBytes( SELF_TEST_X_ACCEL, &(selfTest[0]), 1 ); // X-axis accel self-test results
    MPUReadBytes( SELF_TEST_Y_ACCEL, &(selfTest[1]), 1 ); // Y-axis accel self-test results
    MPUReadBytes( SELF_TEST_Z_ACCEL, &(selfTest[2]), 1 ); // Z-axis accel self-test results
    MPUReadBytes( SELF_TEST_X_GYRO, &(selfTest[3]), 1 );  // X-axis gyro self-test results
    MPUReadBytes( SELF_TEST_Y_GYRO, &(selfTest[4]), 1 );  // Y-axis gyro self-test results
    MPUReadBytes( SELF_TEST_Z_GYRO, &(selfTest[5]), 1 );  // Z-axis gyro self-test results

    /* Retrieve factory self-test value from self-test code reads */
    factoryTrim[0] = (float)(2620/1<<FS)*(powf( 1.01f , ((float)selfTest[0] - 1.0f) )); // FT[Xa] factory trim calculation
    factoryTrim[1] = (float)(2620/1<<FS)*(powf( 1.01f , ((float)selfTest[1] - 1.0f) )); // FT[Ya] factory trim calculation
    factoryTrim[2] = (float)(2620/1<<FS)*(powf( 1.01f , ((float)selfTest[2] - 1.0f) )); // FT[Za] factory trim calculation
    factoryTrim[3] = (float)(2620/1<<FS)*(powf( 1.01f , ((float)selfTest[3] - 1.0f) )); // FT[Xg] factory trim calculation
    factoryTrim[4] = (float)(2620/1<<FS)*(powf( 1.01f , ((float)selfTest[4] - 1.0f) )); // FT[Yg] factory trim calculation
    factoryTrim[5] = (float)(2620/1<<FS)*(powf( 1.01f , ((float)selfTest[5] - 1.0f) )); // FT[Zg] factory trim calculation
 
    /* Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
     * To get percent, must multiply by 100
     */
    for ( ii = 0; ii < 3; ii++ ) 
    {
        deviation[ii]   = 100.0f*((float)(aSTAvg[ii] - aAvg[ii]))/factoryTrim[ii] - 100.0f;   // Report percent differences
        deviation[ii+3] = 100.0f*((float)(gSTAvg[ii] - gAvg[ii]))/factoryTrim[ii+3] - 100.0f; // Report percent differences
   }
}

/************************************************************************************
 * The call-back function for updateing IMU data (i.e., platform pose
 * and linear acceleration).
 * This function should be called on every SAMPLE_RATE trig period.
 ************************************************************************************/
static void simple_high_pass( vector_t* data, vector_t* state )
{
  state->x = (fEmaCoefficient * data->x) + ((1.0f - fEmaCoefficient) * state->x);  /* Run EMA (i.e., low-pass) */
  data->x -= state->x;          /* Calculate the high-pass signal by subtracting data with low-pass */

  state->y = (fEmaCoefficient * data->y) + ((1.0f - fEmaCoefficient) * state->y);  /* Run EMA (i.e., low-pass) */
  data->y -= state->y;          /* Calculate the high-pass signal by subtracting data with low-pass */

  state->z = (fEmaCoefficient * data->z) + ((1.0f - fEmaCoefficient) * state->z);  /* Run EMA (i.e., low-pass) */
  data->z -= state->z;          /* Calculate the high-pass signal by subtracting data with low-pass */
}

static void UpdateData(FAR void *arg)
{
    uint32_t now;
    int16_t rawAcelTempGy[7];
    vector_t t_acc, t_v, t_omega;
    float ax, ay, az, gx, gy, gz; // variables to hold latest sensor data values 
    float tmp;
    float sin_roll, cos_roll, sin_pitch, cos_pitch;
    int   retval;

    UNUSED(arg);

    /* Re-enable sensor update */
    if( work_queue(MPU_WORK_QUEUE, &worker, UpdateData, NULL, SAMPLE_PERIOD) != OK )
    {
        _err("Unable to register IMU to work queue\n");
    }

    if( i2c_bus == NULL )
    {
        /* I2C bus has lose, Re-initialize it */
        _info( "Reinitialize i2c\n" );
        if( ( i2c_bus = stm32_i2cbus_initialize(I2C_CHANNEL) ) == NULL )
        {
            _err("Failed to init i2c\n");
        }
        return; /* Wait until the next loop before reading first data */
    }

    if( !bMPUReady )
    {
        /* If MPU has not initialized, the do it */
        if( InitMPU9250() != OK )
        {
            _err("Failed to initialize IMU\n");
            stm32_i2cbus_uninitialize( i2c_bus );
            i2c_bus = NULL;
            return;
        }
        else
        {
            bMPUReady = true;
        }
    }
    retval = ReadMPU9250Data( rawAcelTempGy );
    if( retval != OK )
    {
        _err("Failed to read MPU9250. Reset i2c\n");
        i2c_bus->ops->reset(i2c_bus);
        (void)stm32_i2cbus_uninitialize( i2c_bus );
        i2c_bus = NULL;
        bMPUReady = false;
        return;
    }

    /* Currently, we leave out the bias */

    /* Now we'll calculate the accleration value into actual g's */
    /*ax = (float)rawAcelTempGy[0]*A_FACTOR - fAccelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)rawAcelTempGy[1]*A_FACTOR - fAccelBias[1];   
    az = (float)rawAcelTempGy[2]*A_FACTOR - fAccelBias[2];  
    */

    /* Get acceleration in G (9.8N = 1G) */
    ax = (float)rawAcelTempGy[0] * A_FACTOR;  /* get actual g value, this depends on scale being set */
    ay = (float)rawAcelTempGy[1] * A_FACTOR;   
    az = (float)rawAcelTempGy[2] * A_FACTOR;  
   
    /* Calculate the gyro value into actual rad per second */
    gx = (float)rawAcelTempGy[4] * G_FACTOR * DEG_TO_RAD;  /* get actual gyro value, this depends on scale being set */
    gy = (float)rawAcelTempGy[5] * G_FACTOR * DEG_TO_RAD;  
    gz = (float)rawAcelTempGy[6] * G_FACTOR * DEG_TO_RAD;   

    //_info("A=[%d|%d|%d] - G=[%d|%d|%d]\n", rawAcelTempGy[0], rawAcelTempGy[1], rawAcelTempGy[2], rawAcelTempGy[4], rawAcelTempGy[5], rawAcelTempGy[6]);

    /* Update time period */
    now = millis();
    tick_count += (now - lastupdate);
    delta_s = (float)(now - lastupdate) * 0.001f;  /* Convert to second */
    lastupdate = now;

    /* Median filter to reduce noise */
    median_filter( &t_acc, -ax, -ay, -az );

    /* Apply comlement filter with NED system */
    complement_filter( t_acc.x, t_acc.y, t_acc.z, gx, gy, gz );
    //_info("a=[%f|%f|%f] - g=[%f|%f|%f] - t=[%f|%f|%f]\n",t_acc.x,t_acc.y,t_acc.z,gx,gy,gz,xTiltAngle.x,xTiltAngle.y,xTiltAngle.z);

    /* Extract real gravity from tilt angle according to
     * the calculation from :
     * https://github.com/KalebKE/AccelerationExplorer/wiki/Sensor-Fusion-Linear-Acceleration
     * It is just the concatenate off roll then pitch (swapping order should give the same value)
     */
    sin_roll = sinf( xTiltAngle.x );
    cos_roll = cosf( xTiltAngle.x );
    sin_pitch = sinf( xTiltAngle.y );
    cos_pitch = cosf( xTiltAngle.y );
    gravity.x = sin_pitch;
    gravity.y = cos_pitch * sin_roll;
    gravity.z = cos_pitch * cos_roll;

    /*
     * Normally, the bias of accelerometer is negative meaning that when
     * the sensor place horizontally flat (accel +z is up). The measurement
     * is 1G pointing in the opposite direction of +z (i.e., down). Also,
     * all other axes. Therefore, we negate all value to get correct bias
     * according to the gravity.
     */
    t_acc.x -= gravity.x;
    t_acc.y -= gravity.y;
    t_acc.z -= gravity.z;
    simple_high_pass( &t_acc, &a_state );

    t_omega.x = gx;
    t_omega.y = gy;
    t_omega.z = gz;
    simple_high_pass( &t_omega, &o_state );

#define G_TO_MM_SSQ         (9800.0f)   /* 1G = 9.8m/s^2 = 9800mm/s^2 */

    /* Before storinn the output, let integrate the value */
    if( integration_enabled >= INTEGRATION_WAIT_PERIOD )
    {
        tmp = 0.5f * delta_s;   /* Common integration factor */

        /* Integrate acceleration to velocity. */
        t_v.x = velo.x + (accel.x + t_acc.x) * G_TO_MM_SSQ * tmp;  /* Also transtate acceleration from G to mm/second^2 */
        t_v.y = velo.y + (accel.y + t_acc.y) * G_TO_MM_SSQ * tmp;
        t_v.z = velo.z + (accel.z + t_acc.z) * G_TO_MM_SSQ * tmp;
        simple_high_pass( &t_v, &v_state );

        /* Integrate velocity to displacement */
        disp.x = disp.x + (velo.x + t_v.x) * tmp;
        disp.y = disp.y + (velo.y + t_v.y) * tmp;
        disp.z = disp.z + (velo.z + t_v.z) * tmp;
        simple_high_pass( &disp, &d_state );

        /* Store current velocity */
        velo.x = t_v.x;
        velo.y = t_v.y;
        velo.z = t_v.z;

        /* Integrate omega to theta */
        theta.x = theta.x + (omega.x + t_omega.x) * tmp;
        theta.y = theta.y + (omega.y + t_omega.y) * tmp;
        theta.z = theta.z + (omega.z + t_omega.z) * tmp;
        simple_high_pass( &theta, &t_state );
    }

    /* Store the processed data */
    accel.x = t_acc.x;
    accel.y = t_acc.y;
    accel.z = t_acc.z;
    omega.x = t_omega.x;
    omega.y = t_omega.y;
    omega.z = t_omega.z;

    /* For debug information, generate log every 0.5 second */
    if( tick_count >= 1000 )
    {
        if( integration_enabled < INTEGRATION_WAIT_PERIOD )
            integration_enabled++;
        tick_count = 0;
/*        _info( " - Raw accel x=%f, y=%f, z=%f\n", ax, ay, az );
        _info( " - Raw gyro x=%f, y=%f, z=%f\n", gx, gy, gz );
        _info( " - Gravity vector x=%f, y=%f, z=%f\n", a_state.x, a_state.y, a_state.z );
        _info( " - Linear accel x=%f, y=%f, z=%f\n", accel.x, accel.y, accel.z );
        _info( " - Linear velocity x=%f, y=%f, z=%f\n", velo.x, velo.y, velo.z );
        _info( " - Linear disp x=%f, y=%f, z=%f\n", disp.x, disp.y, disp.z );
        _info( " - angular velocity x=%f, y=%f, z=%f\n", omega.x, omega.y, omega.z );
        _info( " - angular disp x=%f, y=%f, z=%f\n", theta.x, theta.y, theta.z );
*/    }
}

/************************************************************************************
 * Driver interface
 ************************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     imu_open(FAR struct file *filep);
static int     imu_close(FAR struct file *filep);
static ssize_t imu_read(FAR struct file *filep, FAR char *buffer, size_t buflen);
static ssize_t imu_write(FAR struct file *filep, FAR const char *buffer, size_t buflen);
static int     imu_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_imuops =
{
    imu_open,  /* open */
    imu_close, /* close */
    imu_read,  /* read */
    imu_write, /* write */
    NULL,        /* seek */
    imu_ioctl, /* ioctl */
    NULL         /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL      /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/************************************************************************************
 * Name: imu_open
 *
 * Description:
 *   This function is called whenever the motor device is opened.
 *
 ************************************************************************************/

static int imu_open(FAR struct file *filep)
{
    /* Do nothing, just return OK */
    UNUSED(filep);

    return OK;
}

/************************************************************************************
 * Name: imu_close
 *
 * Description:
 *   This function is called when the motor device is closed.
 *
 ************************************************************************************/

static int imu_close(FAR struct file *filep)
{
    /* Do nothing, just return OK */
    UNUSED(filep);

    return OK;
}

/************************************************************************************
 * Name: imu_read
 *
 * Description:
*   A dummy read method.  This is provided only to satsify the VFS layer.
  *
 ************************************************************************************/

static ssize_t imu_read(FAR struct file *filep, FAR char *buf, size_t buflen)
{
    /* return nothing */
    UNUSED(filep);
    UNUSED(buf);
    UNUSED(buflen);

    return 0;
}

/************************************************************************************
 * Name: imu_write
 *
 * Description:
 *   A dummy write method.  This is provided only to satsify the VFS layer.
 *
 ************************************************************************************/

static ssize_t imu_write(FAR struct file *filep, FAR const char *buf, size_t buflen)
{
    /* Return a failure */
    UNUSED(filep);
    UNUSED(buf);
    UNUSED(buflen);

    return -EPERM;
}

/************************************************************************************
 * Name: imu_ioctl
 *
 * Description:
 *   The standard ioctl method.  This is where ALL of the motor work is done.
 *
 ************************************************************************************/

static int imu_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
    int     ret;
    float*  fp;

    UNUSED(filep);

//    _info("IMU Cmd: %d arg: %ld\n", cmd, arg);
    ret = OK;

    switch (cmd)
    {
        case IMU_CMD_GET_SAMPLE_RATE: /* Arg: Pointer to uint32_t */
            *((uint32_t*)arg) = SAMPLE_RATE;
            break;

        case IMU_CMD_GET_TILT: /* Arg: Array float[3] for x/y/z */
            fp = (float*)arg;
            fp[0] = xTiltAngle.x;
            fp[1] = xTiltAngle.y;
            fp[2] = xTiltAngle.z;
            break;

        case IMU_CMD_GET_GRAVITY: /* Arg: Array float[3] for x/y/z */
            fp = (float*)arg;
            fp[0] = gravity.x;
            fp[1] = gravity.y;
            fp[2] = gravity.z;
            break;

        case IMU_CMD_GET_LIN_ACCEL: /* Arg: Array float[3] for x/y/z */
            fp = (float*)arg;
            fp[0] = accel.x;
            fp[1] = accel.y;
            fp[2] = accel.z;
            break;

        case IMU_CMD_GET_LIN_VELO: /* Arg: Array float[3] for x/y/z */
            fp = (float*)arg;
            fp[0] = velo.x;
            fp[1] = velo.y;
            fp[2] = velo.z;
            break;

        case IMU_CMD_GET_LIN_DISP: /* Arg: Array float[3] for x/y/z */
            fp = (float*)arg;
            fp[0] = disp.x;
            fp[1] = disp.y;
            fp[2] = disp.z;
            break;

        case IMU_CMD_GET_ANG_VELO: /* Arg: Array float[3] for x/y/z */
            fp = (float*)arg;
            fp[0] = omega.x;
            fp[1] = omega.y;
            fp[2] = omega.z;
            break;

        case IMU_CMD_GET_ANG_DISP: /* Arg: Array float[3] for x/y/z */
            fp = (float*)arg;
            fp[0] = theta.x;
            fp[1] = theta.y;
            fp[2] = theta.z;
            break;

        case IMU_CMD_SET_FILTER_COEF: /* Arg: Pointer to float */
            fEmaCoefficient = *((float*)arg);
            break;

        /* Any unrecognized IOCTL commands might be platform-specific ioctl commands */
        default:
            _err("Unrecognized cmd: %d arg: %ld\n", cmd, arg);
            ret = -ENOTSUP;
            break;
    }

  return ret;
}

/************************************************************************************
 * Pubic Functions
 ************************************************************************************/
int imu_initialize(void)
{
    float deviation[6];

    _info("Initialize IMU:..");

    accel.x = accel.y = accel.z = 0.0f;
    velo.x = velo.y = velo.z = 0.0f;
    disp.x = disp.y = disp.z = 0.0f;
    omega.x = omega.y = omega.z = 0.0f;
    theta.x = theta.y = theta.z = 0.0f;
    a_state.x = a_state.y = a_state.z = 0.0f;
    v_state.x = v_state.y = v_state.z = 0.0f;
    d_state.x = d_state.y = d_state.z = 0.0f;
    o_state.x = o_state.y = o_state.z = 0.0f;
    t_state.x = t_state.y = t_state.z = 0.0f;

    /* 1. Init I2C */
    if( ( i2c_bus = stm32_i2cbus_initialize(I2C_CHANNEL) ) == NULL )
    {
        _err("Failed to init i2c\n");
        return -ENOTSUP;
    }

    usleep(2000000);    /* Sleep 1 s for I2C to stabilized */
    /* 2. Init the sensors */
    MPUSelfTest( deviation );
    for( int i = 0; i < 3; i++ )
    {
        _info( "Accelerometer axis %d has deviation %f\n", i, deviation[i] );
        _info( "Gyroscope axis %d has deviation %f\n", i, deviation[i+3] );
    }
    if( InitMPU9250() != OK )
    {
        _err("Failed to initialize IMU\n");
        stm32_i2cbus_uninitialize( i2c_bus );
        i2c_bus = NULL;
        return -ENOTSUP;
    }

    /* 3. Register timer event */
    if( work_queue(MPU_WORK_QUEUE, &worker, UpdateData, NULL, SAMPLE_PERIOD) != OK )
    {
        _err("Unable to register IMU to work queue\n");
        stm32_i2cbus_uninitialize( i2c_bus );
        i2c_bus = NULL;
        return -ENOTSUP;
    }

    /* 4. Register the driver  */
    if( register_driver( "/dev/imu", &g_imuops, 0666, NULL ) != OK )
    {
        _err("Unable to register /dev/imu\n");
        stm32_i2cbus_uninitialize( i2c_bus );
        i2c_bus = NULL;
        return -ENOTSUP;
    }

    bMPUReady = true;

  _info("DONE\n");
  return OK;
}
