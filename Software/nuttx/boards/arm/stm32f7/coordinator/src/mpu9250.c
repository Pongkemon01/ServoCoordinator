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

#define I2C_SPEED               100000  /* I2C Clock speed */

#define MPU9250_ADDRESS         (0x68)  /* AD0 = 0 */
#define AK8963_ADDRESS          (0x0C)  /* Address for direct access */

#define SAMPLE_RATE             100     /* Sensor sample rate (Hz.) max = 1000 */

#define SAMPLE_PERIOD           (1000000/(CONFIG_USEC_PER_TICK * SAMPLE_RATE))

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

typedef struct
{
    float x, y, z;
}vector_t;

typedef struct
{
    float w, x, y, z;    
}quaternion_t;

static const struct i2c_config_s   mpu_i2c_config =
{
    .frequency  = I2C_SPEED,
    .address    = MPU9250_ADDRESS,
    .addrlen    = 7                    /* 7-bit address */
};
static const struct i2c_config_s   compass_i2c_config =
{
    .frequency  = I2C_SPEED,
    .address    = AK8963_ADDRESS,
    .addrlen    = 7                    /* 7-bit address */
};
static struct i2c_master_s*  i2c_bus = NULL;

static uint32_t lastupdate = 0, tick_count = 0;
static float delta_s;    /* Time period used for integration */

static float fMagneticAdjFactor[3];

#define INTEGRATION_WAIT_PERIOD  15  /* The amount of second waiting for the integration function to activate */
static unsigned int integration_enabled = 0;
static vector_t accel; /* Acceleration (acceleration with gravity component subtracted) */
static vector_t velo;  /* Velocity gotten from integrate the acel */
static vector_t disp;  /* Diaplacement gotten from integrate the velo */
static vector_t a_state, v_state, d_state;  /* State for high-pass filter */

static quaternion_t quaternion = 
{   .w = 1.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f };   /* Current orientation */

static struct work_s worker;       /* kernel worker queue data */

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
static int COMWrite1Byte( uint8_t addr, uint8_t data )
{
    uint8_t buffer[2];
    buffer[0] = addr;
    buffer[1] = data;
    return( i2c_write( i2c_bus, &compass_i2c_config, buffer, 2 ) );
}

/************************************************************************************
 * Funtion : MPUReadByte and COMReadBytes
 * Read bytes of data from MPU (address 0x68) or Compass (address 0x0C)
 ************************************************************************************/
static inline int MPUReadBytes( uint8_t addr, uint8_t* data, uint8_t len )
{
    return( i2c_writeread( i2c_bus, &mpu_i2c_config, &addr, 1, data, len ) );
}
static inline int COMReadBytes( uint8_t addr, uint8_t* data, uint8_t len )
{
    return( i2c_writeread( i2c_bus, &compass_i2c_config, &addr, 1, data, len ) );
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

    destination[0] = (int16_t)(((uint16_t)rawData[0] << 8U) | ((uint16_t)rawData[1])) ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = (int16_t)(((uint16_t)rawData[2] << 8U) | ((uint16_t)rawData[3])) ;  
    destination[2] = (int16_t)(((uint16_t)rawData[4] << 8U) | ((uint16_t)rawData[5])) ; 
    destination[3] = (int16_t)(((uint16_t)rawData[6] << 8U) | ((uint16_t)rawData[7])) ;   
    destination[4] = (int16_t)(((uint16_t)rawData[8] << 8U) | ((uint16_t)rawData[9])) ;  
    destination[5] = (int16_t)(((uint16_t)rawData[10] << 8U) | ((uint16_t)rawData[11])) ;  
    destination[6] = (int16_t)(((uint16_t)rawData[12] << 8U) | ((uint16_t)rawData[13])) ; 

    return OK;
}

/*------------------*/
static int ReadMagData( int16_t* destination )
{
    uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
    int     retval;

    retval = COMReadBytes( AK8963_XOUT_L, rawData, 7 ); 
    if( retval != OK )
    {
        _err("Failed to read magneto meter data\n");
        return retval;
    }

    if( rawData[6] & 0x08 ) // Check if magnetic sensor overflow set, if not then report data
    {
        _err("Magneto overflow\n");
        return -EOVERFLOW;
    }
    else
    {
        destination[0] = (int16_t)(((uint16_t)rawData[1] << 8U) | ((uint16_t)rawData[0])) ;  // Turn the MSB and LSB into a signed 16-bit value
        destination[1] = (int16_t)(((uint16_t)rawData[3] << 8U) | ((uint16_t)rawData[2])) ;  // Data stored as little Endian
        destination[2] = (int16_t)(((uint16_t)rawData[5] << 8U) | ((uint16_t)rawData[4])) ; 
        return OK;
    }
}

/************************************************************************************
 * Funtion : InitAK8963 and InitMPU9250
 * Initialize devices.
 ************************************************************************************/
static int InitAK8963(void)
{    
    uint8_t data[3]; 
    int     retval;

    retval = COMWrite1Byte( AK8963_CNTL, AK8963_MODE_PWRDN ); /* Power down magnetometer */
    if( retval != OK )
        return retval;
    usleep(10000);

    /* First extract the factory calibration for each magnetometer axis */
    retval = COMWrite1Byte( AK8963_CNTL, AK8963_MODE_ROMACC ); /* Gain access to fuse ROM data */
    if( retval != OK )
        return retval;
    usleep(10000);

    retval = COMReadBytes( AK8963_ASAX, data, 3 ); /* Read the x-, y-, and z-axis calibration values */
    if( retval != OK )
        return retval;
    fMagneticAdjFactor[0] = (float)( data[0] - 128 )/256.0f + 1.0f;   /* Setting the adjustment factor values */
    fMagneticAdjFactor[1] = (float)( data[1] - 128 )/256.0f + 1.0f;  
    fMagneticAdjFactor[2] = (float)( data[2] - 128 )/256.0f + 1.0f; 

    retval = COMWrite1Byte( AK8963_CNTL, AK8963_MODE_PWRDN ); /* Power down magnetometer */
    if( retval != OK )
        return retval;
    usleep(10000);

    /* Configure the magnetometer for 100Hz continous read with 16-bit resolution */
    return( COMWrite1Byte( AK8963_CNTL, ( AK8963_MODE_CON100HZ | M_RES ) ) );
}

/***********************************************************************************/
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
    retval = MPUWrite1Byte( INT_PIN_CFG, 0x02 ); /* Cuurently we don't have interrupt */
    if( retval != OK )
        return retval;

    /* Initialize the magnetometer AK8963 */
    return( InitAK8963() );
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
 * Implementation of Sebastian Madgwick's "...efficient orientation filter for... 
 * inertial/magnetic sensor arrays" (see http://www.x-io.co.uk/category/open-source/ 
 * for examples and more details) which fuses acceleration, rotation rate, 
 * and magnetic moments to produce a quaternion-based estimate of absolute
 * device orientation -- which can be converted to yaw, pitch, and roll. Useful for 
 * stabilizing quadcopters, etc. The performance of the orientation filter is at least 
 * as good as conventional Kalman-based filtering algorithms but is much less 
 * computationally intensive ---it can be performed on a 3.3 V Pro Mini operating 
 * at 8 MHz!
 ************************************************************************************/

/* global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System) */
/* 
 * There is a tradeoff in the beta parameter between accuracy and response speed.
 * In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError 
 * of 2.7 degrees/s) was found to give optimal accuracy. However, with this value, 
 * the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
 * Subsequent changes also require a longish lag time to a stable output, 
 * not fast enough for a quadcopter or robot car! By increasing beta (GyroMeasError) 
 * by about a factor of fifteen, the response time constant is reduced to ~2 sec
 * I haven't noticed any reduction in solution accuracy. This is essentially 
 * the I coefficient in a PID control sense; the bigger the feedback coefficient, 
 *the faster the solution converges, usually at the expense of accuracy. 
 * In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
 */
//static float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
static float beta = 45.0f * DEG_TO_RAD * 0.866f;   // gyroscope measurement error in rads/s (start at 45 deg/s)
/* compute zeta */
/* the other free parameter in the Madgwick scheme usually set to a small or zero value */
//static float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   
//static float zeta = 0.866 * 0.0f  * DEG_TO_RAD;   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)

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

/*-----------------------------------------------------------------------------------
 * Helper function : Fast square-root
 * See: https://github.com/zrho/Carbon/blob/master/libc/src/math/sqrt.c
 *----------------------------------------------------------------------------------*/
static float FastSqrt( float x )
{
    int32_t i;
    float y;

    /* Floats + bit manipulation = +inf fun! */
    i = *((int32_t *) & x);
    i = 0x1fc00000 + (i >> 1);
    y = *((float *)&i);

    /* The next line can be repeated any number of times to increase accuracy */
    y = 0.5f * (y + x / y);
    y = 0.5f * (y + x / y); /* 2nd iteration to improve accuracy */

    return y;
}

static void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
    float q1 = quaternion.w, q2 = quaternion.x;
    float q3 = quaternion.y, q4 = quaternion.z;   // short name local variable for readability
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    if( (ax == 0) && (ay == 0) && (az == 0) )
        return; // handle NaN
    norm = InvSqrt(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    if( (mx == 0) && (my == 0) && (mz == 0) )
        return; // handle NaN
    norm = InvSqrt(mx * mx + my * my + mz * mz);
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = FastSqrt(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm = InvSqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * delta_s;
    q2 += qDot2 * delta_s;
    q3 += qDot3 * delta_s;
    q4 += qDot4 * delta_s;
    norm = InvSqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    quaternion.w = q1 * norm;
    quaternion.x = q2 * norm;
    quaternion.y = q3 * norm;
    quaternion.z = q4 * norm;
}

/************************************************************************************
 * The call-back function for updateing IMU data (i.e., platform pose
 * and linear acceleration).
 * This function should be called on every SAMPLE_RATE trig period.
 ************************************************************************************/
static void q_multiply( quaternion_t* res, quaternion_t* q1, quaternion_t* q2 )
{
    res->w = (q1->w*q2->w) - (q1->x*q2->x) - (q1->y*q2->y) - (q1->z*q2->z);
    res->x = (q1->w*q2->x) + (q1->x*q2->w) + (q1->y*q2->z) - (q1->z*q2->y);
    res->y = (q1->w*q2->y) - (q1->x*q2->z) + (q1->y*q2->w) + (q1->z*q2->x);
    res->z = (q1->w*q2->z) + (q1->x*q2->y) - (q1->y*q2->x) + (q1->z*q2->w);
}

#define EMA_A   0.3f
static void simple_high_pass( vector_t* data, vector_t* state )
{
  state->x = (EMA_A * data->x) + ((1.0f - EMA_A) * state->x);  /* Run EMA (i.e., low-pass) */
  data->x -= state->x;          /* Calculate the high-pass signal by subtracting data with low-pass */

  state->y = (EMA_A * data->y) + ((1.0f - EMA_A) * state->y);  /* Run EMA (i.e., low-pass) */
  data->y -= state->y;          /* Calculate the high-pass signal by subtracting data with low-pass */

  state->z = (EMA_A * data->z) + ((1.0f - EMA_A) * state->z);  /* Run EMA (i.e., low-pass) */
  data->z -= state->z;          /* Calculate the high-pass signal by subtracting data with low-pass */
}

static void UpdateData(FAR void *arg)
{
    uint32_t now;
    int16_t rawAcelTempGy[7], rawMag[3];
    //float a31, a32, a33;
    quaternion_t conj, r, res;
    vector_t acc, v;
    float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
    int     retval;

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
            return;
        }
    }

    retval = ReadMPU9250Data( rawAcelTempGy );
    if( retval != OK )
    {
        _err("Failed to read MPU9250. Release i2c\n");
        (void)stm32_i2cbus_uninitialize( i2c_bus );
        i2c_bus = NULL;
        return;
    }

    /* Currently, we leave out the bias */

    /* Now we'll calculate the accleration value into actual g's */
    /*ax = (float)rawAcelTempGy[0]*A_FACTOR - fAccelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)rawAcelTempGy[1]*A_FACTOR - fAccelBias[1];   
    az = (float)rawAcelTempGy[2]*A_FACTOR - fAccelBias[2];  
    */
    ax = (float)rawAcelTempGy[0] * A_FACTOR;  /* get actual g value, this depends on scale being set */
    ay = (float)rawAcelTempGy[1] * A_FACTOR;   
    az = (float)rawAcelTempGy[2] * A_FACTOR;  
   
    /* Calculate the gyro value into actual degrees per second */
    gx = (float)rawAcelTempGy[4] * G_FACTOR * DEG_TO_RAD;  /* get actual gyro value, this depends on scale being set */
    gy = (float)rawAcelTempGy[5] * G_FACTOR * DEG_TO_RAD;  
    gz = (float)rawAcelTempGy[6] * G_FACTOR * DEG_TO_RAD;   

    retval = ReadMagData( rawMag );
    if( retval != OK )
    {
        _err("Failed to read Magneto meter. Release i2c\n");
        (void)stm32_i2cbus_uninitialize( i2c_bus );
        i2c_bus = NULL;
        return;
    }
     /*mx = (float)magCount[0]*M_FACTOR*fMagneticAdjFactor[0] - magBias[0];
    my = (float)magCount[1]*M_FACTOR*fMagneticAdjFactor[1] - magBias[1];  
    mz = (float)magCount[2]*M_FACTOR*fMagneticAdjFactor[2] - magBias[2];  
    mx *= magScale[0];
    my *= magScale[1];
    mz *= magScale[2]; 
    */
    mx = (float)rawMag[0] * M_FACTOR * fMagneticAdjFactor[0];  // get actual magnetometer value, this depends on scale being set
    my = (float)rawMag[1] * M_FACTOR * fMagneticAdjFactor[1];  
    mz = (float)rawMag[2] * M_FACTOR * fMagneticAdjFactor[2];  

    /* Update time period */
    now = millis();
    tick_count += (now - lastupdate);
    delta_s = (float)(now - lastupdate) * 0.001f;  /* Convert to second */
    lastupdate = now;

    /*
     * THe convension of orientation in the Earth frame used here is 
     * NED (Norith-Easr-Down), where positive x-axis points to the north, 
     * positive y-axis points to the east, and positive z-axis points downward.
     * NED convension specifies the sensor direction with respect to
     * local frame to be positive x-axis pointed to the head of the vehicle,
     * positive y-axis pointed to the right of the vehicle, and positive
     * z-axis pointed down.
     * Data from each sensors (i.e., Gyro, Accelero, and Magneto) must
     * be adjusted to align with the NED direction before fed into 
     * fusion function.
     * ---------------------------------------------------------------
     * For the MPU9250 SparkFun breakout mounted underneath the PTTEP
     * robot platform (up-side down mounting). The circuit mounted makes 
     * the accelerometer axes already aligned to the correct direction. 
     * This mounting also makes the rotational data from gyroscope algned
     * with the convensional NED standard. However, the magnetic sensor
     * has different alignment. With this mounting configuration, x-axis 
     * is swapped with y-axis. The z-axis, also, points to the reverse direction.
     * Therefore, we need to realign the parameters passed to 
     * the fusion function as followed.
     */

    /* Firstly, adjust the sensor data to conform with NED coordinate and
     * the accelerometer should have the bias of 1g downward
     */
    /* 
     * Normally, the bias of accelerometer is negative meaning that when
     * the sensor place horizontally flat (accel +z is up). The measurement
     * is 1G pointing in the opposite direction of +z (i.e., down). Also,
     * all other axes. Therefore, we negate all value to get correct bias
     * according to the gravity.
     */
    ax = -ax;
    ay = -ay;
    az = -az;
    MadgwickQuaternionUpdate( ax, ay, az, gx, gy, gz, my, mx, -mz );

    /* Define output variables from updated quaternion---these are Tait-Bryan 
     * angles, commonly used in aircraft orientation. In this coordinate system, 
     * the positive z-axis is down toward Earth. 
     * ---
     * Yaw is the angle between Sensor x-axis and Earth magnetic North 
     ๕ (or true North if corrected for local declination, looking down from 
     ๕ the sensor positive yaw is counterclockwise.
     * ---
     * Pitch is angle between sensor x-axis and Earth ground plane, 
     * toward the Earth is positive, up toward the sky is negative.
     * ---
     * Roll is angle between sensor y-axis and Earth ground plane, y-axis up 
     * is positive roll.
     * ---
     * These arise from the definition of the homogeneous rotation matrix 
     * constructed from quaternions. Tait-Bryan angles as well as Euler angles 
     * are non-commutative; that is, the get the correct orientation 
     * the rotations must be applied in the correct order which for this 
     * configuration is yaw, pitch, and then roll. For more see 
     * http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles 
     * which has additional links.
     */

    /* compensate the accelerometer readings from gravity.  */
    /*a31 = 2.0f * ( ( quaternion.w * quaternion.x ) + ( quaternion.y * quaternion.x ) );
    a32 = 2.0f * ( ( quaternion.x * quaternion.z ) - ( quaternion.w * quaternion.y ) );
    a33 = ( quaternion.w * quaternion.w ) - ( quaternion.x * quaternion.x ) - 
            ( quaternion.y * quaternion.y ) + ( quaternion.z * quaternion.z );

    if( avg_count == 10 )
    {
        accel.x = accel_acc.x / 10.0f;
        accel.y = accel_acc.y / 10.0f;
        accel.z = accel_acc.z / 10.0f;

        accel_acc.x = 0.0f;
        accel_acc.y = 0.0f;
        accel_acc.z = 0.0f;

        avg_count = 0;
    }
    accel_acc.x += (ax - a32);
    accel_acc.y += (ay - a31);
    accel_acc.z += (az - a33);
    avg_count++; */

    r.w = 0;
    r.x = ax;
    r.y = ay;
    r.z = az;
    conj.w = quaternion.w;
    conj.x = -(quaternion.x);
    conj.y = -(quaternion.y);
    conj.z = -(quaternion.z);

    /* This just worked!!!! I think the order of multiplication should be q'aq but it seems that 
     * the correct order is qaq' meaning that q can be used to compensate the rotation directly */
    q_multiply( &res, &r, &conj );
    q_multiply( &r, &quaternion, &res );
    r.z -= 1.0f;    /* Remove graviry */
    acc.x = ROUND_1(r.x);
    acc.y = ROUND_1(r.y);
    acc.z = ROUND_1(r.z);
    simple_high_pass( &acc, &a_state );

    /* Before storinn the output, let integrate the value */
    if( integration_enabled >= INTEGRATION_WAIT_PERIOD )
    {
        v.x = velo.x + (accel.x + acc.x) * 4.9f * delta_s; /* 4.9f comes from 0.5 * g (9.8) */
        v.y = velo.y + (accel.y + acc.y) * 4.9f * delta_s;
        v.z = velo.z + (accel.z + acc.z) * 4.9f * delta_s;
        simple_high_pass( &v, &v_state );

        disp.x = disp.x + (velo.x + v.x) * 0.5f * delta_s;
        disp.y = disp.y + (velo.y + v.y) * 0.5f * delta_s;
        disp.z = disp.z + (velo.z + v.z) * 0.5f * delta_s;
        simple_high_pass( &disp, &d_state );

        velo.x = v.x;
        velo.y = v.y;
        velo.z = v.z;
    }

    /* Store the result */
    accel.x = acc.x;
    accel.y = acc.y;
    accel.z = acc.z;


    /* For debug information, generate log every 0.5 second */
    if( tick_count >= 1000 )
    {
        if( integration_enabled < INTEGRATION_WAIT_PERIOD )
            integration_enabled++;
        tick_count = 0;
        //_info( " - Quaternion: %f + %fi + %fj + %fk\n", quaternion.w, quaternion.x, quaternion.y, quaternion.z );
        //_info( " - Raw accel x=%f, y=%f, z=%f\n", ax, ay, az );
        //_info( " - Linear accel x=%f, y=%f, z=%f\n", accel.x, accel.y, accel.z );
        //_info( " - Linear velocity x=%f, y=%f, z=%f\n", velo.x, velo.y, velo.z );
        //_info( " - Linear disp x=%f, y=%f, z=%f\n", disp.x, disp.y, disp.z );
    }
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

    _info("IMU Cmd: %d arg: %ld\n", cmd, arg);

    switch (cmd)
    {
        case IMU_CMD_GET_SAMPLE_RATE: /* Arg: Pointer to uint32_t */
            *((uint32_t*)arg) = SAMPLE_RATE;
            break;

        case IMU_CMD_GET_QUATERNION: /* Arg: Array float[4] */
            fp = (float*)arg;
            fp[0] = quaternion.w;
            fp[1] = quaternion.x;
            fp[2] = quaternion.y;
            fp[3] = quaternion.z;
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
    a_state.x = a_state.y = a_state.z = 0.0f;
    v_state.x = v_state.y = v_state.z = 0.0f;
    d_state.x = d_state.y = d_state.z = 0.0f;

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
  _info("DONE\n");
  return OK;
}
