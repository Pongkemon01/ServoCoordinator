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

#define A_FACTOR    ( 2.0f / 32768.0f )

#define F_PI        3.1415926535897932384626433832795029f
#define EARTH_G     9.8f
#define DEG_TO_RAD  ( F_PI / 180.0f )
#define RAD_TO_DEG  ( 180.0f / F_PI )

#define ROUND_1(_x)      ((float)( (int)(_x * 10.0f + 0.5f) ) * 0.1f)
#define ROUND_2(_x)      ((float)( (int)(_x * 100.0f + 0.5f) ) * 0.01f)

#define DMP_SAMPLE_RATE         100     /* Sensor sample rate for DMP (default = max = 200) */
#define DMP_SAMPLE_SCALE        ( ( 200 / DMP_SAMPLE_RATE ) - 1 ) /* Scaling factor */
#define GYRO_SF                 (46850825LL * 200 / DMP_SAMPLE_RATE) /* Gyro scale factor */

#define SAMPLE_PERIOD           (1000000/(CONFIG_USEC_PER_TICK * DMP_SAMPLE_RATE))



/* DMP memory address */

/* These defines are copied from dmpDefaultMPU6050.c in the general MPL
 * releases. These defines may change for each DMP image, so be sure to modify
 * these values when switching to a new image.
 */
#define CFG_LP_QUAT             (2712)
#define END_ORIENT_TEMP         (1866)
#define CFG_27                  (2742)
#define CFG_20                  (2224)
#define CFG_23                  (2745)
#define CFG_FIFO_ON_EVENT       (2690)
#define END_PREDICTION_UPDATE   (1761)
#define CGNOTICE_INTR           (2620)
#define X_GRT_Y_TMP             (1358)
#define CFG_DR_INT              (1029)
#define CFG_AUTH                (1035)
#define UPDATE_PROP_ROT         (1835)
#define END_COMPARE_Y_X_TMP2    (1455)
#define SKIP_X_GRT_Y_TMP        (1359)
#define SKIP_END_COMPARE        (1435)

#define GYRO_MOUNT_MATRIX_CONFIG_SIGN  (1088)
#define ACCEL_MOUNT_MATRIX_CONFIG_SIGN                  (1073)
#define ACCEL_MOUNT_MATRIX_CONFIG                  (1066)
#define GYRO_MOUNT_MATRIX_CONFIG                  (1062)

#define END_COMPARE_Y_X_TMP3    (1434)
#define FCFG_6                  (1106)
#define FLAT_STATE_END          (1713)
#define SWING_END_4             (1616)
#define SWING_END_2             (1565)
#define SWING_END_3             (1587)
#define SWING_END_1             (1550)
#define X6_LPQ_EN                   (2718)
#define CFG_15                  (2727)
#define CFG_16                  (2746)
#define CFG_EXT_GYRO_BIAS       (1189)
#define END_COMPARE_Y_X_TMP     (1407)
#define DO_NOT_UPDATE_PROP_ROT  (1839)
#define CFG_7                   (1205)
#define FLAT_STATE_END_TEMP     (1683)
#define END_COMPARE_Y_X         (1484)
#define SKIP_SWING_END_1        (1551)
#define SKIP_SWING_END_3        (1588)
#define SKIP_SWING_END_2        (1566)
#define TILTG75_START           (1672)
#define CFG_6                   (2753)
#define TILTL75_END             (1669)
#define END_ORIENT              (1884)
#define CFG_FLICK_IN            (2573)
#define TILTL75_START           (1643)
#define CFG_MOTION_BIAS         (1208)
#define X_GRT_Y                 (1408)
#define TEMPLABEL               (2324)
#define CFG_ANDROID_ORIENT_INT  (1853)
#define CFG_GYRO_RAW_DATA       (2722)
#define X_GRT_Y_TMP2            (1379)

#define FIFO_RATE_DIV                  (22+512)
#define D_0_24                  (24+512)

#define D_0_36                  (36)
#define D_0_52                  (52)
#define D_0_96                  (96)
#define GYRO_INT_SCALE_FACTOR                 (104)
#define D_0_108                 (108)
#define D_0_163                 (163)
#define D_0_188                 (188)
#define D_0_192                 (192)
#define D_0_224                 (224)
#define D_0_228                 (228)
#define D_0_232                 (232)
#define D_0_236                 (236)

#define D_1_2                   (256 + 2)
#define D_1_4                   (256 + 4)
#define D_1_8                   (256 + 8)
#define D_1_10                  (256 + 10)
#define D_1_24                  (256 + 24)
#define D_1_28                  (256 + 28)
#define D_1_36                  (256 + 36)
#define D_1_40                  (256 + 40)
#define D_1_44                  (256 + 44)
#define D_1_72                  (256 + 72)
#define D_1_74                  (256 + 74)
#define D_1_79                  (256 + 79)
#define D_1_88                  (256 + 88)
#define D_1_90                  (256 + 90)
#define D_1_92                  (256 + 92)
#define D_1_96                  (256 + 96)
#define D_1_98                  (256 + 98)
#define D_1_106                 (256 + 106)
#define D_1_108                 (256 + 108)
#define D_1_112                 (256 + 112)
#define D_1_128                 (256 + 144)
#define D_1_152                 (256 + 12)
#define D_1_160                 (256 + 160)
#define D_1_176                 (256 + 176)
#define D_1_178                 (256 + 178)
#define D_1_218                 (256 + 218)
#define D_1_232                 (256 + 232)
#define D_1_236                 (256 + 236)
#define D_1_240                 (256 + 240)
#define D_1_244                 (256 + 244)
#define D_1_250                 (256 + 250)
#define D_1_252                 (256 + 252)
#define D_2_12                  (512 + 12)
#define D_2_96                  (512 + 96)
#define D_2_108                 (512 + 108)
#define D_2_208                 (512 + 208)
#define D_2_224                 (512 + 224)
#define D_2_236                 (512 + 236)
#define D_2_244                 (512 + 244)
#define D_2_248                 (512 + 248)
#define D_2_252                 (512 + 252)

#define CPASS_BIAS_X            (35 * 16 + 4)
#define CPASS_BIAS_Y            (35 * 16 + 8)
#define CPASS_BIAS_Z            (35 * 16 + 12)
#define CPASS_MTX_00            (36 * 16)
#define CPASS_MTX_01            (36 * 16 + 4)
#define CPASS_MTX_02            (36 * 16 + 8)
#define CPASS_MTX_10            (36 * 16 + 12)
#define CPASS_MTX_11            (37 * 16)
#define CPASS_MTX_12            (37 * 16 + 4)
#define CPASS_MTX_20            (37 * 16 + 8)
#define CPASS_MTX_21            (37 * 16 + 12)
#define CPASS_MTX_22            (43 * 16 + 12)
#define D_EXT_GYRO_BIAS_X       (61 * 16)
#define D_EXT_GYRO_BIAS_Y       (61 * 16) + 4
#define D_EXT_GYRO_BIAS_Z       (61 * 16) + 8
#define D_ACT0                  (40 * 16)
#define D_ACSX                  (40 * 16 + 4)
#define D_ACSY                  (40 * 16 + 8)
#define D_ACSZ                  (40 * 16 + 12)

#define FLICK_MSG               (45 * 16 + 4)
#define FLICK_COUNTER           (45 * 16 + 8)
#define FLICK_LOWER             (45 * 16 + 12)
#define FLICK_UPPER             (46 * 16 + 12)

#define D_AUTH_OUT              (992)
#define D_AUTH_IN               (996)
#define D_AUTH_A                (1000)
#define D_AUTH_B                (1004)

#define D_PEDSTD_BP_B           (768 + 0x1C)
#define D_PEDSTD_HP_A           (768 + 0x78)
#define D_PEDSTD_HP_B           (768 + 0x7C)
#define D_PEDSTD_BP_A4          (768 + 0x40)
#define D_PEDSTD_BP_A3          (768 + 0x44)
#define D_PEDSTD_BP_A2          (768 + 0x48)
#define D_PEDSTD_BP_A1          (768 + 0x4C)
#define D_PEDSTD_INT_THRSH      (768 + 0x68)
#define D_PEDSTD_CLIP           (768 + 0x6C)
#define D_PEDSTD_SB             (768 + 0x28)
#define D_PEDSTD_SB_TIME        (768 + 0x2C)
#define D_PEDSTD_PEAKTHRSH      (768 + 0x98)
#define D_PEDSTD_TIML           (768 + 0x2A)
#define D_PEDSTD_TIMH           (768 + 0x2E)
#define D_PEDSTD_PEAK           (768 + 0X94)
#define D_PEDSTD_STEPCTR        (768 + 0x60)
#define D_PEDSTD_TIMECTR        (964)
#define D_PEDSTD_DECI           (768 + 0xA0)

#define D_HOST_NO_MOT           (976)
#define D_ACCEL_BIAS            (660)

#define D_ORIENT_GAP            (76)

#define D_TILT0_H               (48)
#define D_TILT0_L               (50)
#define D_TILT1_H               (52)
#define D_TILT1_L               (54)
#define D_TILT2_H               (56)
#define D_TILT2_L               (58)
#define D_TILT3_H               (60)
#define D_TILT3_L               (62)

#define DMP_CODE_SIZE           (3062)

static const unsigned char dmp_memory[DMP_CODE_SIZE] = {
    /* bank # 0 */
    0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00,
    0x00, 0x65, 0x00, 0x54, 0xff, 0xef, 0x00, 0x00, 0xfa, 0x80, 0x00, 0x0b, 0x12, 0x82, 0x00, 0x01,
    0x03, 0x0c, 0x30, 0xc3, 0x0e, 0x8c, 0x8c, 0xe9, 0x14, 0xd5, 0x40, 0x02, 0x13, 0x71, 0x0f, 0x8e,
    0x38, 0x83, 0xf8, 0x83, 0x30, 0x00, 0xf8, 0x83, 0x25, 0x8e, 0xf8, 0x83, 0x30, 0x00, 0xf8, 0x83,
    0xff, 0xff, 0xff, 0xff, 0x0f, 0xfe, 0xa9, 0xd6, 0x24, 0x00, 0x04, 0x00, 0x1a, 0x82, 0x79, 0xa1,
    0x00, 0x00, 0x00, 0x3c, 0xff, 0xff, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x38, 0x83, 0x6f, 0xa2,
    0x00, 0x3e, 0x03, 0x30, 0x40, 0x00, 0x00, 0x00, 0x02, 0xca, 0xe3, 0x09, 0x3e, 0x80, 0x00, 0x00,
    0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
    0x00, 0x0c, 0x00, 0x00, 0x00, 0x0c, 0x18, 0x6e, 0x00, 0x00, 0x06, 0x92, 0x0a, 0x16, 0xc0, 0xdf,
    0xff, 0xff, 0x02, 0x56, 0xfd, 0x8c, 0xd3, 0x77, 0xff, 0xe1, 0xc4, 0x96, 0xe0, 0xc5, 0xbe, 0xaa,
    0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x0b, 0x2b, 0x00, 0x00, 0x16, 0x57, 0x00, 0x00, 0x03, 0x59,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1d, 0xfa, 0x00, 0x02, 0x6c, 0x1d, 0x00, 0x00, 0x00, 0x00,
    0x3f, 0xff, 0xdf, 0xeb, 0x00, 0x3e, 0xb3, 0xb6, 0x00, 0x0d, 0x22, 0x78, 0x00, 0x00, 0x2f, 0x3c,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x42, 0xb5, 0x00, 0x00, 0x39, 0xa2, 0x00, 0x00, 0xb3, 0x65,
    0xd9, 0x0e, 0x9f, 0xc9, 0x1d, 0xcf, 0x4c, 0x34, 0x30, 0x00, 0x00, 0x00, 0x50, 0x00, 0x00, 0x00,
    0x3b, 0xb6, 0x7a, 0xe8, 0x00, 0x64, 0x00, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    /* bank # 1 */
    0x10, 0x00, 0x00, 0x00, 0x10, 0x00, 0xfa, 0x92, 0x10, 0x00, 0x22, 0x5e, 0x00, 0x0d, 0x22, 0x9f,
    0x00, 0x01, 0x00, 0x00, 0x00, 0x32, 0x00, 0x00, 0xff, 0x46, 0x00, 0x00, 0x63, 0xd4, 0x00, 0x00,
    0x10, 0x00, 0x00, 0x00, 0x04, 0xd6, 0x00, 0x00, 0x04, 0xcc, 0x00, 0x00, 0x04, 0xcc, 0x00, 0x00,
    0x00, 0x00, 0x10, 0x72, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x06, 0x00, 0x02, 0x00, 0x05, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x05, 0x00, 0x64, 0x00, 0x20, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x03, 0x00,
    0x00, 0x00, 0x00, 0x32, 0xf8, 0x98, 0x00, 0x00, 0xff, 0x65, 0x00, 0x00, 0x83, 0x0f, 0x00, 0x00,
    0xff, 0x9b, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0xb2, 0x6a, 0x00, 0x02, 0x00, 0x00,
    0x00, 0x01, 0xfb, 0x83, 0x00, 0x68, 0x00, 0x00, 0x00, 0xd9, 0xfc, 0x00, 0x7c, 0xf1, 0xff, 0x83,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x00, 0x00, 0x64, 0x03, 0xe8, 0x00, 0x64, 0x00, 0x28,
    0x00, 0x00, 0x00, 0x25, 0x00, 0x00, 0x00, 0x00, 0x16, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
    0x00, 0x00, 0x10, 0x00, 0x00, 0x2f, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf4, 0x00, 0x00, 0x10, 0x00,
    /* bank # 2 */
    0x00, 0x28, 0x00, 0x00, 0xff, 0xff, 0x45, 0x81, 0xff, 0xff, 0xfa, 0x72, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x00, 0x05, 0x00, 0x05, 0xba, 0xc6, 0x00, 0x47, 0x78, 0xa2,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x14,
    0x00, 0x00, 0x25, 0x4d, 0x00, 0x2f, 0x70, 0x6d, 0x00, 0x00, 0x05, 0xae, 0x00, 0x0c, 0x02, 0xd0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x64, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x0e,
    0x00, 0x00, 0x0a, 0xc7, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0xff, 0xff, 0xff, 0x9c,
    0x00, 0x00, 0x0b, 0x2b, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x64,
    0xff, 0xe5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    /* bank # 3 */
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x24, 0x26, 0xd3,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x10, 0x00, 0x96, 0x00, 0x3c,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0c, 0x0a, 0x4e, 0x68, 0xcd, 0xcf, 0x77, 0x09, 0x50, 0x16, 0x67, 0x59, 0xc6, 0x19, 0xce, 0x82,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xd7, 0x84, 0x00, 0x03, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc7, 0x93, 0x8f, 0x9d, 0x1e, 0x1b, 0x1c, 0x19,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x03, 0x18, 0x85, 0x00, 0x00, 0x40, 0x00,
    0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x67, 0x7d, 0xdf, 0x7e, 0x72, 0x90, 0x2e, 0x55, 0x4c, 0xf6, 0xe6, 0x88,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /* bank # 4 */
    0xd8, 0xdc, 0xb4, 0xb8, 0xb0, 0xd8, 0xb9, 0xab, 0xf3, 0xf8, 0xfa, 0xb3, 0xb7, 0xbb, 0x8e, 0x9e,
    0xae, 0xf1, 0x32, 0xf5, 0x1b, 0xf1, 0xb4, 0xb8, 0xb0, 0x80, 0x97, 0xf1, 0xa9, 0xdf, 0xdf, 0xdf,
    0xaa, 0xdf, 0xdf, 0xdf, 0xf2, 0xaa, 0xc5, 0xcd, 0xc7, 0xa9, 0x0c, 0xc9, 0x2c, 0x97, 0xf1, 0xa9,
    0x89, 0x26, 0x46, 0x66, 0xb2, 0x89, 0x99, 0xa9, 0x2d, 0x55, 0x7d, 0xb0, 0xb0, 0x8a, 0xa8, 0x96,
    0x36, 0x56, 0x76, 0xf1, 0xba, 0xa3, 0xb4, 0xb2, 0x80, 0xc0, 0xb8, 0xa8, 0x97, 0x11, 0xb2, 0x83,
    0x98, 0xba, 0xa3, 0xf0, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xb2, 0xb9, 0xb4, 0x98, 0x83, 0xf1,
    0xa3, 0x29, 0x55, 0x7d, 0xba, 0xb5, 0xb1, 0xa3, 0x83, 0x93, 0xf0, 0x00, 0x28, 0x50, 0xf5, 0xb2,
    0xb6, 0xaa, 0x83, 0x93, 0x28, 0x54, 0x7c, 0xf1, 0xb9, 0xa3, 0x82, 0x93, 0x61, 0xba, 0xa2, 0xda,
    0xde, 0xdf, 0xdb, 0x81, 0x9a, 0xb9, 0xae, 0xf5, 0x60, 0x68, 0x70, 0xf1, 0xda, 0xba, 0xa2, 0xdf,
    0xd9, 0xba, 0xa2, 0xfa, 0xb9, 0xa3, 0x82, 0x92, 0xdb, 0x31, 0xba, 0xa2, 0xd9, 0xba, 0xa2, 0xf8,
    0xdf, 0x85, 0xa4, 0xd0, 0xc1, 0xbb, 0xad, 0x83, 0xc2, 0xc5, 0xc7, 0xb8, 0xa2, 0xdf, 0xdf, 0xdf,
    0xba, 0xa0, 0xdf, 0xdf, 0xdf, 0xd8, 0xd8, 0xf1, 0xb8, 0xaa, 0xb3, 0x8d, 0xb4, 0x98, 0x0d, 0x35,
    0x5d, 0xb2, 0xb6, 0xba, 0xaf, 0x8c, 0x96, 0x19, 0x8f, 0x9f, 0xa7, 0x0e, 0x16, 0x1e, 0xb4, 0x9a,
    0xb8, 0xaa, 0x87, 0x2c, 0x54, 0x7c, 0xba, 0xa4, 0xb0, 0x8a, 0xb6, 0x91, 0x32, 0x56, 0x76, 0xb2,
    0x84, 0x94, 0xa4, 0xc8, 0x08, 0xcd, 0xd8, 0xb8, 0xb4, 0xb0, 0xf1, 0x99, 0x82, 0xa8, 0x2d, 0x55,
    0x7d, 0x98, 0xa8, 0x0e, 0x16, 0x1e, 0xa2, 0x2c, 0x54, 0x7c, 0x92, 0xa4, 0xf0, 0x2c, 0x50, 0x78,
    /* bank # 5 */
    0xf1, 0x84, 0xa8, 0x98, 0xc4, 0xcd, 0xfc, 0xd8, 0x0d, 0xdb, 0xa8, 0xfc, 0x2d, 0xf3, 0xd9, 0xba,
    0xa6, 0xf8, 0xda, 0xba, 0xa6, 0xde, 0xd8, 0xba, 0xb2, 0xb6, 0x86, 0x96, 0xa6, 0xd0, 0xf3, 0xc8,
    0x41, 0xda, 0xa6, 0xc8, 0xf8, 0xd8, 0xb0, 0xb4, 0xb8, 0x82, 0xa8, 0x92, 0xf5, 0x2c, 0x54, 0x88,
    0x98, 0xf1, 0x35, 0xd9, 0xf4, 0x18, 0xd8, 0xf1, 0xa2, 0xd0, 0xf8, 0xf9, 0xa8, 0x84, 0xd9, 0xc7,
    0xdf, 0xf8, 0xf8, 0x83, 0xc5, 0xda, 0xdf, 0x69, 0xdf, 0x83, 0xc1, 0xd8, 0xf4, 0x01, 0x14, 0xf1,
    0xa8, 0x82, 0x4e, 0xa8, 0x84, 0xf3, 0x11, 0xd1, 0x82, 0xf5, 0xd9, 0x92, 0x28, 0x97, 0x88, 0xf1,
    0x09, 0xf4, 0x1c, 0x1c, 0xd8, 0x84, 0xa8, 0xf3, 0xc0, 0xf9, 0xd1, 0xd9, 0x97, 0x82, 0xf1, 0x29,
    0xf4, 0x0d, 0xd8, 0xf3, 0xf9, 0xf9, 0xd1, 0xd9, 0x82, 0xf4, 0xc2, 0x03, 0xd8, 0xde, 0xdf, 0x1a,
    0xd8, 0xf1, 0xa2, 0xfa, 0xf9, 0xa8, 0x84, 0x98, 0xd9, 0xc7, 0xdf, 0xf8, 0xf8, 0xf8, 0x83, 0xc7,
    0xda, 0xdf, 0x69, 0xdf, 0xf8, 0x83, 0xc3, 0xd8, 0xf4, 0x01, 0x14, 0xf1, 0x98, 0xa8, 0x82, 0x2e,
    0xa8, 0x84, 0xf3, 0x11, 0xd1, 0x82, 0xf5, 0xd9, 0x92, 0x50, 0x97, 0x88, 0xf1, 0x09, 0xf4, 0x1c,
    0xd8, 0x84, 0xa8, 0xf3, 0xc0, 0xf8, 0xf9, 0xd1, 0xd9, 0x97, 0x82, 0xf1, 0x49, 0xf4, 0x0d, 0xd8,
    0xf3, 0xf9, 0xf9, 0xd1, 0xd9, 0x82, 0xf4, 0xc4, 0x03, 0xd8, 0xde, 0xdf, 0xd8, 0xf1, 0xad, 0x88,
    0x98, 0xcc, 0xa8, 0x09, 0xf9, 0xd9, 0x82, 0x92, 0xa8, 0xf5, 0x7c, 0xf1, 0x88, 0x3a, 0xcf, 0x94,
    0x4a, 0x6e, 0x98, 0xdb, 0x69, 0x31, 0xda, 0xad, 0xf2, 0xde, 0xf9, 0xd8, 0x87, 0x95, 0xa8, 0xf2,
    0x21, 0xd1, 0xda, 0xa5, 0xf9, 0xf4, 0x17, 0xd9, 0xf1, 0xae, 0x8e, 0xd0, 0xc0, 0xc3, 0xae, 0x82,
    /* bank # 6 */
    0xc6, 0x84, 0xc3, 0xa8, 0x85, 0x95, 0xc8, 0xa5, 0x88, 0xf2, 0xc0, 0xf1, 0xf4, 0x01, 0x0e, 0xf1,
    0x8e, 0x9e, 0xa8, 0xc6, 0x3e, 0x56, 0xf5, 0x54, 0xf1, 0x88, 0x72, 0xf4, 0x01, 0x15, 0xf1, 0x98,
    0x45, 0x85, 0x6e, 0xf5, 0x8e, 0x9e, 0x04, 0x88, 0xf1, 0x42, 0x98, 0x5a, 0x8e, 0x9e, 0x06, 0x88,
    0x69, 0xf4, 0x01, 0x1c, 0xf1, 0x98, 0x1e, 0x11, 0x08, 0xd0, 0xf5, 0x04, 0xf1, 0x1e, 0x97, 0x02,
    0x02, 0x98, 0x36, 0x25, 0xdb, 0xf9, 0xd9, 0x85, 0xa5, 0xf3, 0xc1, 0xda, 0x85, 0xa5, 0xf3, 0xdf,
    0xd8, 0x85, 0x95, 0xa8, 0xf3, 0x09, 0xda, 0xa5, 0xfa, 0xd8, 0x82, 0x92, 0xa8, 0xf5, 0x78, 0xf1,
    0x88, 0x1a, 0x84, 0x9f, 0x26, 0x88, 0x98, 0x21, 0xda, 0xf4, 0x1d, 0xf3, 0xd8, 0x87, 0x9f, 0x39,
    0xd1, 0xaf, 0xd9, 0xdf, 0xdf, 0xfb, 0xf9, 0xf4, 0x0c, 0xf3, 0xd8, 0xfa, 0xd0, 0xf8, 0xda, 0xf9,
    0xf9, 0xd0, 0xdf, 0xd9, 0xf9, 0xd8, 0xf4, 0x0b, 0xd8, 0xf3, 0x87, 0x9f, 0x39, 0xd1, 0xaf, 0xd9,
    0xdf, 0xdf, 0xf4, 0x1d, 0xf3, 0xd8, 0xfa, 0xfc, 0xa8, 0x69, 0xf9, 0xf9, 0xaf, 0xd0, 0xda, 0xde,
    0xfa, 0xd9, 0xf8, 0x8f, 0x9f, 0xa8, 0xf1, 0xcc, 0xf3, 0x98, 0xdb, 0x45, 0xd9, 0xaf, 0xdf, 0xd0,
    0xf8, 0xd8, 0xf1, 0x8f, 0x9f, 0xa8, 0xca, 0xf3, 0x88, 0x09, 0xda, 0xaf, 0x8f, 0xcb, 0xf8, 0xd8,
    0xf2, 0xad, 0x97, 0x8d, 0x0c, 0xd9, 0xa5, 0xdf, 0xf9, 0xba, 0xa6, 0xf3, 0xfa, 0xf4, 0x12, 0xf2,
    0xd8, 0x95, 0x0d, 0xd1, 0xd9, 0xba, 0xa6, 0xf3, 0xfa, 0xda, 0xa5, 0xf2, 0xc1, 0xba, 0xa6, 0xf3,
    0xdf, 0xd8, 0xf1, 0xba, 0xb2, 0xb6, 0x86, 0x96, 0xa6, 0xd0, 0xca, 0xf3, 0x49, 0xda, 0xa6, 0xcb,
    0xf8, 0xd8, 0xb0, 0xb4, 0xb8, 0xd8, 0xad, 0x84, 0xf2, 0xc0, 0xdf, 0xf1, 0x8f, 0xcb, 0xc3, 0xa8,
    /* bank # 7 */
    0xb2, 0xb6, 0x86, 0x96, 0xc8, 0xc1, 0xcb, 0xc3, 0xf3, 0xb0, 0xb4, 0x88, 0x98, 0xa8, 0x21, 0xdb,
    0x71, 0x8d, 0x9d, 0x71, 0x85, 0x95, 0x21, 0xd9, 0xad, 0xf2, 0xfa, 0xd8, 0x85, 0x97, 0xa8, 0x28,
    0xd9, 0xf4, 0x08, 0xd8, 0xf2, 0x8d, 0x29, 0xda, 0xf4, 0x05, 0xd9, 0xf2, 0x85, 0xa4, 0xc2, 0xf2,
    0xd8, 0xa8, 0x8d, 0x94, 0x01, 0xd1, 0xd9, 0xf4, 0x11, 0xf2, 0xd8, 0x87, 0x21, 0xd8, 0xf4, 0x0a,
    0xd8, 0xf2, 0x84, 0x98, 0xa8, 0xc8, 0x01, 0xd1, 0xd9, 0xf4, 0x11, 0xd8, 0xf3, 0xa4, 0xc8, 0xbb,
    0xaf, 0xd0, 0xf2, 0xde, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xd8, 0xf1, 0xb8, 0xf6,
    0xb5, 0xb9, 0xb0, 0x8a, 0x95, 0xa3, 0xde, 0x3c, 0xa3, 0xd9, 0xf8, 0xd8, 0x5c, 0xa3, 0xd9, 0xf8,
    0xd8, 0x7c, 0xa3, 0xd9, 0xf8, 0xd8, 0xf8, 0xf9, 0xd1, 0xa5, 0xd9, 0xdf, 0xda, 0xfa, 0xd8, 0xb1,
    0x85, 0x30, 0xf7, 0xd9, 0xde, 0xd8, 0xf8, 0x30, 0xad, 0xda, 0xde, 0xd8, 0xf2, 0xb4, 0x8c, 0x99,
    0xa3, 0x2d, 0x55, 0x7d, 0xa0, 0x83, 0xdf, 0xdf, 0xdf, 0xb5, 0x91, 0xa0, 0xf6, 0x29, 0xd9, 0xfb,
    0xd8, 0xa0, 0xfc, 0x29, 0xd9, 0xfa, 0xd8, 0xa0, 0xd0, 0x51, 0xd9, 0xf8, 0xd8, 0xfc, 0x51, 0xd9,
    0xf9, 0xd8, 0x79, 0xd9, 0xfb, 0xd8, 0xa0, 0xd0, 0xfc, 0x79, 0xd9, 0xfa, 0xd8, 0xa1, 0xf9, 0xf9,
    0xf9, 0xf9, 0xf9, 0xa0, 0xda, 0xdf, 0xdf, 0xdf, 0xd8, 0xa1, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xac,
    0xde, 0xf8, 0xad, 0xde, 0x83, 0x93, 0xac, 0x2c, 0x54, 0x7c, 0xf1, 0xa8, 0xdf, 0xdf, 0xdf, 0xf6,
    0x9d, 0x2c, 0xda, 0xa0, 0xdf, 0xd9, 0xfa, 0xdb, 0x2d, 0xf8, 0xd8, 0xa8, 0x50, 0xda, 0xa0, 0xd0,
    0xde, 0xd9, 0xd0, 0xf8, 0xf8, 0xf8, 0xdb, 0x55, 0xf8, 0xd8, 0xa8, 0x78, 0xda, 0xa0, 0xd0, 0xdf,
    /* bank # 8 */
    0xd9, 0xd0, 0xfa, 0xf8, 0xf8, 0xf8, 0xf8, 0xdb, 0x7d, 0xf8, 0xd8, 0x9c, 0xa8, 0x8c, 0xf5, 0x30,
    0xdb, 0x38, 0xd9, 0xd0, 0xde, 0xdf, 0xa0, 0xd0, 0xde, 0xdf, 0xd8, 0xa8, 0x48, 0xdb, 0x58, 0xd9,
    0xdf, 0xd0, 0xde, 0xa0, 0xdf, 0xd0, 0xde, 0xd8, 0xa8, 0x68, 0xdb, 0x70, 0xd9, 0xdf, 0xdf, 0xa0,
    0xdf, 0xdf, 0xd8, 0xf1, 0xa8, 0x88, 0x90, 0x2c, 0x54, 0x7c, 0x98, 0xa8, 0xd0, 0x5c, 0x38, 0xd1,
    0xda, 0xf2, 0xae, 0x8c, 0xdf, 0xf9, 0xd8, 0xb0, 0x87, 0xa8, 0xc1, 0xc1, 0xb1, 0x88, 0xa8, 0xc6,
    0xf9, 0xf9, 0xda, 0x36, 0xd8, 0xa8, 0xf9, 0xda, 0x36, 0xd8, 0xa8, 0xf9, 0xda, 0x36, 0xd8, 0xa8,
    0xf9, 0xda, 0x36, 0xd8, 0xa8, 0xf9, 0xda, 0x36, 0xd8, 0xf7, 0x8d, 0x9d, 0xad, 0xf8, 0x18, 0xda,
    0xf2, 0xae, 0xdf, 0xd8, 0xf7, 0xad, 0xfa, 0x30, 0xd9, 0xa4, 0xde, 0xf9, 0xd8, 0xf2, 0xae, 0xde,
    0xfa, 0xf9, 0x83, 0xa7, 0xd9, 0xc3, 0xc5, 0xc7, 0xf1, 0x88, 0x9b, 0xa7, 0x7a, 0xad, 0xf7, 0xde,
    0xdf, 0xa4, 0xf8, 0x84, 0x94, 0x08, 0xa7, 0x97, 0xf3, 0x00, 0xae, 0xf2, 0x98, 0x19, 0xa4, 0x88,
    0xc6, 0xa3, 0x94, 0x88, 0xf6, 0x32, 0xdf, 0xf2, 0x83, 0x93, 0xdb, 0x09, 0xd9, 0xf2, 0xaa, 0xdf,
    0xd8, 0xd8, 0xae, 0xf8, 0xf9, 0xd1, 0xda, 0xf3, 0xa4, 0xde, 0xa7, 0xf1, 0x88, 0x9b, 0x7a, 0xd8,
    0xf3, 0x84, 0x94, 0xae, 0x19, 0xf9, 0xda, 0xaa, 0xf1, 0xdf, 0xd8, 0xa8, 0x81, 0xc0, 0xc3, 0xc5,
    0xc7, 0xa3, 0x92, 0x83, 0xf6, 0x28, 0xad, 0xde, 0xd9, 0xf8, 0xd8, 0xa3, 0x50, 0xad, 0xd9, 0xf8,
    0xd8, 0xa3, 0x78, 0xad, 0xd9, 0xf8, 0xd8, 0xf8, 0xf9, 0xd1, 0xa1, 0xda, 0xde, 0xc3, 0xc5, 0xc7,
    0xd8, 0xa1, 0x81, 0x94, 0xf8, 0x18, 0xf2, 0xb0, 0x89, 0xac, 0xc3, 0xc5, 0xc7, 0xf1, 0xd8, 0xb8,
    /* bank # 9 */
    0xb4, 0xb0, 0x97, 0x86, 0xa8, 0x31, 0x9b, 0x06, 0x99, 0x07, 0xab, 0x97, 0x28, 0x88, 0x9b, 0xf0,
    0x0c, 0x20, 0x14, 0x40, 0xb0, 0xb4, 0xb8, 0xf0, 0xa8, 0x8a, 0x9a, 0x28, 0x50, 0x78, 0xb7, 0x9b,
    0xa8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31, 0xf1, 0xbb, 0xab,
    0x88, 0x00, 0x2c, 0x54, 0x7c, 0xf0, 0xb3, 0x8b, 0xb8, 0xa8, 0x04, 0x28, 0x50, 0x78, 0xf1, 0xb0,
    0x88, 0xb4, 0x97, 0x26, 0xa8, 0x59, 0x98, 0xbb, 0xab, 0xb3, 0x8b, 0x02, 0x26, 0x46, 0x66, 0xb0,
    0xb8, 0xf0, 0x8a, 0x9c, 0xa8, 0x29, 0x51, 0x79, 0x8b, 0x29, 0x51, 0x79, 0x8a, 0x24, 0x70, 0x59,
    0x8b, 0x20, 0x58, 0x71, 0x8a, 0x44, 0x69, 0x38, 0x8b, 0x39, 0x40, 0x68, 0x8a, 0x64, 0x48, 0x31,
    0x8b, 0x30, 0x49, 0x60, 0x88, 0xf1, 0xac, 0x00, 0x2c, 0x54, 0x7c, 0xf0, 0x8c, 0xa8, 0x04, 0x28,
    0x50, 0x78, 0xf1, 0x88, 0x97, 0x26, 0xa8, 0x59, 0x98, 0xac, 0x8c, 0x02, 0x26, 0x46, 0x66, 0xf0,
    0x89, 0x9c, 0xa8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31, 0xa9,
    0x88, 0x09, 0x20, 0x59, 0x70, 0xab, 0x11, 0x38, 0x40, 0x69, 0xa8, 0x19, 0x31, 0x48, 0x60, 0x8c,
    0xa8, 0x3c, 0x41, 0x5c, 0x20, 0x7c, 0x00, 0xf1, 0x87, 0x98, 0x19, 0x86, 0xa8, 0x6e, 0x76, 0x7e,
    0xa9, 0x99, 0x88, 0x2d, 0x55, 0x7d, 0xd8, 0xb1, 0xb5, 0xb9, 0xa3, 0xdf, 0xdf, 0xdf, 0xae, 0xd0,
    0xdf, 0xaa, 0xd0, 0xde, 0xf2, 0xab, 0xf8, 0xf9, 0xd9, 0xb0, 0x87, 0xc4, 0xaa, 0xf1, 0xdf, 0xdf,
    0xbb, 0xaf, 0xdf, 0xdf, 0xb9, 0xd8, 0xb1, 0xf1, 0xa3, 0x97, 0x8e, 0x60, 0xdf, 0xb0, 0x84, 0xf2,
    0xc8, 0xf8, 0xf9, 0xd9, 0xde, 0xd8, 0x93, 0x85, 0xf1, 0x4a, 0xb1, 0x83, 0xa3, 0x08, 0xb5, 0x83,
    /* bank # 10 */
    0x9a, 0x08, 0x10, 0xb7, 0x9f, 0x10, 0xd8, 0xf1, 0xb0, 0xba, 0xae, 0xb0, 0x8a, 0xc2, 0xb2, 0xb6,
    0x8e, 0x9e, 0xf1, 0xfb, 0xd9, 0xf4, 0x1d, 0xd8, 0xf9, 0xd9, 0x0c, 0xf1, 0xd8, 0xf8, 0xf8, 0xad,
    0x61, 0xd9, 0xae, 0xfb, 0xd8, 0xf4, 0x0c, 0xf1, 0xd8, 0xf8, 0xf8, 0xad, 0x19, 0xd9, 0xae, 0xfb,
    0xdf, 0xd8, 0xf4, 0x16, 0xf1, 0xd8, 0xf8, 0xad, 0x8d, 0x61, 0xd9, 0xf4, 0xf4, 0xac, 0xf5, 0x9c,
    0x9c, 0x8d, 0xdf, 0x2b, 0xba, 0xb6, 0xae, 0xfa, 0xf8, 0xf4, 0x0b, 0xd8, 0xf1, 0xae, 0xd0, 0xf8,
    0xad, 0x51, 0xda, 0xae, 0xfa, 0xf8, 0xf1, 0xd8, 0xb9, 0xb1, 0xb6, 0xa3, 0x83, 0x9c, 0x08, 0xb9,
    0xb1, 0x83, 0x9a, 0xb5, 0xaa, 0xc0, 0xfd, 0x30, 0x83, 0xb7, 0x9f, 0x10, 0xb5, 0x8b, 0x93, 0xf2,
    0x02, 0x02, 0xd1, 0xab, 0xda, 0xde, 0xd8, 0xf1, 0xb0, 0x80, 0xba, 0xab, 0xc0, 0xc3, 0xb2, 0x84,
    0xc1, 0xc3, 0xd8, 0xb1, 0xb9, 0xf3, 0x8b, 0xa3, 0x91, 0xb6, 0x09, 0xb4, 0xd9, 0xab, 0xde, 0xb0,
    0x87, 0x9c, 0xb9, 0xa3, 0xdd, 0xf1, 0xb3, 0x8b, 0x8b, 0x8b, 0x8b, 0x8b, 0xb0, 0x87, 0xa3, 0xa3,
    0xa3, 0xa3, 0xb2, 0x8b, 0xb6, 0x9b, 0xf2, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3,
    0xa3, 0xf1, 0xb0, 0x87, 0xb5, 0x9a, 0xa3, 0xf3, 0x9b, 0xa3, 0xa3, 0xdc, 0xba, 0xac, 0xdf, 0xb9,
    0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3,
    0xd8, 0xd8, 0xd8, 0xbb, 0xb3, 0xb7, 0xf1, 0xaa, 0xf9, 0xda, 0xff, 0xd9, 0x80, 0x9a, 0xaa, 0x28,
    0xb4, 0x80, 0x98, 0xa7, 0x20, 0xb7, 0x97, 0x87, 0xa8, 0x66, 0x88, 0xf0, 0x79, 0x51, 0xf1, 0x90,
    0x2c, 0x87, 0x0c, 0xa7, 0x81, 0x97, 0x62, 0x93, 0xf0, 0x71, 0x71, 0x60, 0x85, 0x94, 0x01, 0x29,
    /* bank # 11 */
    0x51, 0x79, 0x90, 0xa5, 0xf1, 0x28, 0x4c, 0x6c, 0x87, 0x0c, 0x95, 0x18, 0x85, 0x78, 0xa3, 0x83,
    0x90, 0x28, 0x4c, 0x6c, 0x88, 0x6c, 0xd8, 0xf3, 0xa2, 0x82, 0x00, 0xf2, 0x10, 0xa8, 0x92, 0x19,
    0x80, 0xa2, 0xf2, 0xd9, 0x26, 0xd8, 0xf1, 0x88, 0xa8, 0x4d, 0xd9, 0x48, 0xd8, 0x96, 0xa8, 0x39,
    0x80, 0xd9, 0x3c, 0xd8, 0x95, 0x80, 0xa8, 0x39, 0xa6, 0x86, 0x98, 0xd9, 0x2c, 0xda, 0x87, 0xa7,
    0x2c, 0xd8, 0xa8, 0x89, 0x95, 0x19, 0xa9, 0x80, 0xd9, 0x38, 0xd8, 0xa8, 0x89, 0x39, 0xa9, 0x80,
    0xda, 0x3c, 0xd8, 0xa8, 0x2e, 0xa8, 0x39, 0x90, 0xd9, 0x0c, 0xd8, 0xa8, 0x95, 0x31, 0x98, 0xd9,
    0x0c, 0xd8, 0xa8, 0x09, 0xd9, 0xff, 0xd8, 0x01, 0xda, 0xff, 0xd8, 0x95, 0x39, 0xa9, 0xda, 0x26,
    0xff, 0xd8, 0x90, 0xa8, 0x0d, 0x89, 0x99, 0xa8, 0x10, 0x80, 0x98, 0x21, 0xda, 0x2e, 0xd8, 0x89,
    0x99, 0xa8, 0x31, 0x80, 0xda, 0x2e, 0xd8, 0xa8, 0x86, 0x96, 0x31, 0x80, 0xda, 0x2e, 0xd8, 0xa8,
    0x87, 0x31, 0x80, 0xda, 0x2e, 0xd8, 0xa8, 0x82, 0x92, 0xf3, 0x41, 0x80, 0xf1, 0xd9, 0x2e, 0xd8,
    0xa8, 0x82, 0xf3, 0x19, 0x80, 0xf1, 0xd9, 0x2e, 0xd8, 0x82, 0xac, 0xf3, 0xc0, 0xa2, 0x80, 0x22,
    0xf1, 0xa6, 0x2e, 0xa7, 0x2e, 0xa9, 0x22, 0x98, 0xa8, 0x29, 0xda, 0xac, 0xde, 0xff, 0xd8, 0xa2,
    0xf2, 0x2a, 0xf1, 0xa9, 0x2e, 0x82, 0x92, 0xa8, 0xf2, 0x31, 0x80, 0xa6, 0x96, 0xf1, 0xd9, 0x00,
    0xac, 0x8c, 0x9c, 0x0c, 0x30, 0xac, 0xde, 0xd0, 0xde, 0xff, 0xd8, 0x8c, 0x9c, 0xac, 0xd0, 0x10,
    0xac, 0xde, 0x80, 0x92, 0xa2, 0xf2, 0x4c, 0x82, 0xa8, 0xf1, 0xca, 0xf2, 0x35, 0xf1, 0x96, 0x88,
    0xa6, 0xd9, 0x00, 0xd8, 0xf1, 0xff
};

static const unsigned short sStartAddress = 0x0400;


static int8_t orientationMatrix[3][3] = {
  {1, 0, 0},
  {0, 1, 0},
  {0, 0, 1}
};


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
static struct i2c_master_s*  i2c_bus = NULL;

static uint32_t lastupdate = 0, tick_count = 0;
static float delta_s;    /* Time period used for integration */

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

/************************************************************************************
 * Funtion : MPUWriteBytes and COMWriteBytes
 * Write bytes of data to MPU (address 0x68) or Compass (address 0x0C)
 ************************************************************************************/
static inline int MPUWriteBytes( uint8_t addr, uint8_t* data, uint8_t len )
{
    uint8_t buffer[32];

    DEBUGASSERT(len < 30);  /* Length of buffer is limit to 30 */

    buffer[0] = addr;
    memcpy( &(buffer[1]) , data, len );

    return( i2c_write( i2c_bus, &mpu_i2c_config, buffer, ( len + 1 ) ) );
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
 *  @brief      Write to the DMP memory.
 *  This function prevents I2C writes past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to write.
 *  @param[in]  data        Bytes to write to memory.
 *  @return     0 if successful.
 ************************************************************************************/
static int DMPWriteBytes( uint16_t mem_addr, uint8_t* data, uint8_t length )
{
    uint8_t tmp[2];
    int     retval;

    tmp[0] = (uint8_t)(mem_addr >> 8);
    tmp[1] = (uint8_t)(mem_addr & 0xFF);

    retval = MPUWriteBytes( DMP_BANK, tmp, 2 );
    if ( retval != OK )
        return retval;

    return( MPUWriteBytes( DMP_REG, data, length ) );
}

/************************************************************************************
 *  @brief      Read from the DMP memory.
 *  This function prevents I2C reads past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to read.
 *  @param[out] data        Bytes read from memory.
 *  @return     0 if successful.
************************************************************************************/
static int DMPReadBytes( uint16_t mem_addr, uint8_t* data, uint8_t length )
{
    uint8_t tmp[2];
    int     retval;

    tmp[0] = (uint8_t)(mem_addr >> 8);
    tmp[1] = (uint8_t)(mem_addr & 0xFF);

    retval = MPUWriteBytes( DMP_BANK, tmp, 2 );
    if ( retval != OK )
        return retval;

    return( MPUReadBytes( DMP_REG, data, length ) );
}

/************************************************************************************
 *  @brief  Reset FIFO read/write pointers and DMP and then enable them.
 *  @return 0 if successful.
 ************************************************************************************/
static int DMPResetFIFO(void)
{
    int     retval;

    retval = MPUWrite1Byte( USER_CTRL, 0x0C ); /* Reset both FIFO and DMP */
    if ( retval != OK )
        return retval;
    usleep(50000);
    return( MPUWrite1Byte( USER_CTRL, 0xC0 ) ); /* Enable both FIFO and DMP */
}

/************************************************************************************
 *  @brief      Get one unparsed packet from the FIFO.
 *  This function should be used if the packet is to be parsed elsewhere.
 *  @param[in]  length  Length of one FIFO packet.
 *  @param[in]  data    FIFO packet.
 *  @ return the amount of data remains in FIFO after read ( >= 0 ) or error code (<0)
************************************************************************************/
static int DMPReadFIFO( uint8_t *data, uint16_t length )
{
    uint8_t  tmp[2];
    uint16_t fifo_count;
    int retval;

    retval = MPUReadBytes( FIFO_COUNTH, tmp, 2 );
    if ( retval != OK )
        return retval;

    fifo_count = ((uint16_t)(tmp[0]) << 8) | (uint16_t)(tmp[1]);
    if (fifo_count < length)
    {
        return -EAGAIN; /* Data unavailable */
    }

    if (fifo_count > (512)) 
    {
        /* FIFO is 50% full, better check overflow bit. */
        retval = MPUReadBytes( INT_STATUS, tmp, 1 );
        if ( retval != OK )
            return retval;
        if (tmp[0] & 0x10) 
        {
            DMPResetFIFO();
            return -EOVERFLOW;
        }
    }

    retval = MPUReadBytes( FIFO_R_W, data, length );
    if ( retval != OK )
        return retval;
    return ((fifo_count / length) - 1);
}


/************************************************************************************
 *  @brief      Load and verify DMP image.
 *  @param[in]  length      Length of DMP image.
 *  @param[in]  firmware    DMP code.
 *  @param[in]  start_addr  Starting address of DMP code memory.
 *  @param[in]  sample_rate Fixed sampling rate used when DMP is enabled.
 *  @return     0 if successful.
************************************************************************************/
static int DMPLoadFirmware(void)
{
    uint16_t    ii;
    uint8_t     this_write;
    int         retval;
    /* Must divide evenly into st.hw->bank_size to avoid bank crossings. */
#define LOAD_CHUNK  (16)
#define min(a,b) ((a<b)?a:b)

    uint8_t     cur[LOAD_CHUNK], tmp[2];

    /* Writing firmware */
    for ( ii = 0; ii < DMP_CODE_SIZE; ii += this_write )
    {
        this_write = min( LOAD_CHUNK, DMP_CODE_SIZE - ii );

        /* Writing */
        retval = DMPWriteBytes( ii, (uint8_t*)&( dmp_memory[ii] ), this_write );
        if ( retval != OK )
        {
            return retval;
        }

        /* Read back the previous written */
        retval = DMPReadBytes( ii, cur, this_write );
        if ( retval != OK )
        {
            return retval;
        }

        /* Verify the written data */
        if ( memcmp( ( dmp_memory + ii ), cur, this_write ) )
            return -EINVAL;
    }

    /* Set program start address. */
    tmp[0] = sStartAddress >> 8;
    tmp[1] = sStartAddress & 0xFF;
    return( MPUWriteBytes( DMP_FW_START_H, tmp, 2 ) );
}

/************************************************************************************
 *  @brief      Push gyro and accel orientation to the DMP.
 *  The orientation is represented here as a 3x3 matrix.
 *  @param[in]  orient  Gyro and accel orientation in body frame.
 *  @return     0 if successful.
************************************************************************************/

/* Initial constants from the datasheet */
static const uint8_t gyro_axes[3] = { 0x4C, 0xCD, 0x6C };
static const uint8_t accel_axes[3] = { 0x0C, 0xC9, 0x2C };
static const uint8_t gyro_sign[3] = { 0x36, 0x56, 0x76 };
static const uint8_t accel_sign[3] = { 0x26, 0x46, 0x66 };

/* Helper function to get the position and sign of a matrix row 
 * The return value is valid only 3 LSB where bit 1 and 0 indicate
 * the column number of non-zero value and bit 2 indicates the 
 * sign of the value (1 = negative) */
static int val_position( int8_t *row )
{
    if ( row[0] > 0 )
        return 0;   /* 0b000 */
    else if ( row[0] < 0 )
        return 4;   /* 0b100 */
    else if ( row[1] > 0 )
        return 1;   /* 0b001 */
    else if ( row[1] < 0 )
        return 5;   /* 0b101 */
    else if ( row[2] > 0 )
        return 2;   /* 0b010 */
    else if ( row[2] < 0 )
        return 6;   /* 0b110 */
    else
        return 7;      // error
}

static int DMPSetOrientation(void)
{
    uint8_t data[3], sign[3];
    int     i, v, retval;

    /* Permutate gyrodata */
    for( i = 0; i < 3; i++ )
    {
        v = val_position( orientationMatrix[i] );
        if( v == 7 )
            return -EINVAL;
        data[i] = gyro_axes[ v & 3 ];
        sign[i] = gyro_sign[i];
        if( v & 4 )
            sign[i] |= 1;
    }
    /* Push the result to DMP */
    retval = DMPWriteBytes( GYRO_MOUNT_MATRIX_CONFIG, data, 3 );
    if( retval != OK )
        return retval;
    retval = DMPWriteBytes( GYRO_MOUNT_MATRIX_CONFIG_SIGN, sign, 3 );
    if( retval != OK )
        return retval;

    /* Permutate gyrodata */
    for( i = 0; i < 3; i++ )
    {
        v = val_position( orientationMatrix[i] );
        data[i] = accel_axes[ v & 3 ];
        sign[i] = accel_sign[i];
        if( v & 4 )
            sign[i] |= 1;
    }
    /* Push the result to DMP */
    retval = DMPWriteBytes( ACCEL_MOUNT_MATRIX_CONFIG, data, 3 );
    if( retval != OK )
        return retval;
    return( DMPWriteBytes( ACCEL_MOUNT_MATRIX_CONFIG_SIGN, sign, 3 ) );
}

/************************************************************************************
 * Funtion : DMPInit
 * Initialize and start Digital Motion Processor (DMP) unit.
 ************************************************************************************/
static int DMPInit(void)
{
    uint8_t tmp[16];
    int     retval;

    /* To initia;ize the DMP: (Refer to the relavant application note)
     * 1 Configure Power Management Registers
     *      Write 0x00 to PWR_MGMT_1 (0x6B).
     *      Write 0x00 to PWR_MGMT_2 (0x6C).
     *    Note: This is also the hardware reset value for these registers.
     * 2 Configure Gyroscope Parameters
     *      Write 0x03 to CONFIG (0x1A).
     *      Write 0x18 to GYRO_CONFIG (0x1B).
     *    Sets the cut-off frequency of the Digital Low-Pass Frequency (DLPF) 
     *      filter to 42Hz. Sets the Full Scale Range (FSR) of the gyroscope 
     *      to 2000dps.
     * 3 Configure Accelerometer Parameters
     *      Write 0x00 to ACCEL_CONFIG (0x1C) Sets the Accelerometer FSR to 2g.
     * 4 Configure FIFO and Interrupts
     *      Write 0x00 to FIFO_EN (0x23).
     *      Write 0x00 to INT_ENABLE (0x38).
     *    Defers control of the FIFO and the interrupts from the MPU to the DMP.
     * 5 Reset the FIFO
     *      Write 0x04 to USER_CTRL (0x6A)
     * 6 Configure Sensor Sample Rate
     *      Write 0x04 to SMPLRT_DIV (0x19) Sets sample rate to 200Hz.
     * 7 Load DMP Firmware
     */

    /* Skip step 1 as the reset values are the desired values */
    /* 2. Configure Gyroscope Parameters */
    tmp[0] = 0x03;
    tmp[1] = 0x18;
    retval = MPUWriteBytes( CONFIG, tmp, 2 );
    if( retval != OK )
        return retval;

    /* 3. Configure Accelerometer Parameters */
    tmp[0] = 0;
    retval = MPUWrite1Byte( ACCEL_CONFIG, 0 );
    if( retval != OK )
        return retval;

    /* 4. Defer control and interrupt to DMP */
    retval = MPUWrite1Byte( FIFO_EN, 0 );
    if( retval != OK )
        return retval;
    retval = MPUWrite1Byte( INT_ENABLE, 0 );
    if( retval != OK )
        return retval;

    /* 5. Reset FIFO */
    retval = MPUWrite1Byte( USER_CTRL, 0x04 );
    if( retval != OK )
        return retval;

    /* 6. Set sample rate to 200Hz (Fixed for DMP) */
    retval = MPUWrite1Byte( SMPLRT_DIV, 0x04 );
    if( retval != OK )
        return retval;

    /* 7. Upload firmware */
    retval = DMPLoadFirmware();
    if( retval != OK )
        return retval;

    /* Configure the sensors */
    retval = DMPSetOrientation();
    if( retval != OK )
        return retval;

    /* Enable DMP festures:
       These process consists of writing specific values to DMP
     */
    /* DMP sensor fusion works only with gyro at +-2000dps and accel +-2G */

    /* Set integration scale factor. (Undocumented but found in example) */
    tmp[0] = (uint8_t)( (GYRO_SF >> 24 ) & 0xFF );
    tmp[1] = (uint8_t)( (GYRO_SF >> 16 ) & 0xFF );
    tmp[2] = (uint8_t)( (GYRO_SF >> 8 ) & 0xFF) ;
    tmp[3] = (uint8_t)( GYRO_SF & 0xFF );
    retval = DMPWriteBytes( GYRO_INT_SCALE_FACTOR, tmp, 4 );
    if( retval != OK )
        return retval;

    /* Enable 6-axes quaternion */
    tmp[0] = 0x20;
    tmp[1] = 0x28;
    tmp[2] = 0x30;
    tmp[3] = 0x38;
    retval = DMPWriteBytes( X6_LPQ_EN, tmp, 4 );
    if( retval != OK )
        return retval;

    /* Enable raw accelerometer data. */
    tmp[0] = 0xA3;
    tmp[1] = 0xC0;
    tmp[2] = 0xC8;
    tmp[3] = 0xC2;
    tmp[4] = 0xA3;
    tmp[5] = 0xA3;
    tmp[6] = 0xA3;
    tmp[7] = 0xA3;
    tmp[8] = 0xA3;
    tmp[9] = 0xA3;
    retval = DMPWriteBytes(CFG_15, tmp, 10);
    if( retval != OK )
        return retval;

    /* Disable gesture data */
    tmp[0] = 0xD8;
    retval = DMPWriteBytes(CFG_27, tmp, 1);
    if( retval != OK )
        return retval;

    /* Disable gyro calibration data */
    tmp[0] = 0xb8;
    tmp[1] = 0xaa;
    tmp[2] = 0xaa;
    tmp[3] = 0xaa;
    tmp[4] = 0xb0;
    tmp[5] = 0x88;
    tmp[6] = 0xc3;
    tmp[7] = 0xc5;
    tmp[8] = 0xc7;
    retval = DMPWriteBytes(CFG_MOTION_BIAS, tmp, 9);
    if( retval != OK )
        return retval;

    /* Disable TAP and Android orientation */
    tmp[0] = 0xD8;
    retval = DMPWriteBytes(CFG_ANDROID_ORIENT_INT, tmp, 1);
    if( retval != OK )
        return retval;
    retval = DMPWriteBytes(CFG_20, tmp, 1);
    if( retval != OK )
        return retval;

    /* Set FIFO rate */
    tmp[0] = (uint8_t)( ( DMP_SAMPLE_SCALE >> 8 ) & 0xFF );
    tmp[1] = (uint8_t)( DMP_SAMPLE_SCALE & 0xFF );
    retval = DMPWriteBytes( FIFO_RATE_DIV, tmp, 2 );
    if( retval != OK )
        return retval;
    tmp[0] = 0xFE;
    tmp[1] = 0xF2;
    tmp[2] = 0xAB;
    tmp[3] = 0xC4;
    tmp[4] = 0xAA;
    tmp[5] = 0xF1;
    tmp[6] = 0xDF;
    tmp[7] = 0xDF;
    tmp[8] = 0xBB;
    tmp[9] = 0xAF;
    tmp[10] = 0xDF;
    tmp[11] = 0xDF;
    retval = DMPWriteBytes( CFG_6, tmp, 12 );
    if( retval != OK )
        return retval;

    /* Reset and Enable the DMP */
    return( DMPResetFIFO () );
}

/************************************************************************************
 *  @brief      Get one packet from the FIFO.
 *  If @e sensors does not contain a particular sensor, disregard the data
 *  returned to that pointer.
 *  \n @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  \n INV_WXYZ_QUAT
 *  \n If the FIFO has no new data, @e sensors will be zero.
 *  \n If the FIFO is disabled, @e sensors will be zero and this function will
 *  return a non-zero error code.
 *  @param[out] accel       Accel data in hardware units.
 *  @param[out] quat        3-axis quaternion data in hardware units.
 *  @return     0 if successful.
 ************************************************************************************/
int DMPGetRawSensor(int16_t *acc, int32_t *quat)
{
    uint8_t fifo_data[22];
    int32_t quat_q14[4];
    int32_t quat_mag_sq;
    int retval;

/* Length
    dmp.packet_length = 0;
    if (mask & DMP_FEATURE_SEND_RAW_ACCEL)
        dmp.packet_length += 6;
    if (mask & DMP_FEATURE_SEND_ANY_GYRO)
        dmp.packet_length += 6;
    if (mask & (DMP_FEATURE_LP_QUAT | DMP_FEATURE_6X_LP_QUAT))
        dmp.packet_length += 16;
    if (mask & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT))
        dmp.packet_length += 4;
*/

    /* Get a packet. */
    retval = DMPReadFIFO( fifo_data, 22 );
    if (retval < 0)
        return retval;

    /* Parse DMP packet. */

    /* Quaternion */
    quat[0] = ((int32_t)fifo_data[0] << 24) | ((int32_t)fifo_data[1] << 16) |
            ((int32_t)fifo_data[2] << 8) | (int32_t)fifo_data[3];
    quat[1] = ((int32_t)fifo_data[4] << 24) | ((int32_t)fifo_data[5] << 16) |
            ((int32_t)fifo_data[6] << 8) | (int32_t)fifo_data[7];
    quat[2] = ((int32_t)fifo_data[8] << 24) | ((int32_t)fifo_data[9] << 16) |
            ((int32_t)fifo_data[10] << 8) | (int32_t)fifo_data[11];
    quat[3] = ((int32_t)fifo_data[12] << 24) | ((int32_t)fifo_data[13] << 16) |
            ((int32_t)fifo_data[14] << 8) | (int32_t)fifo_data[15];

    /* We can detect a corrupted FIFO by monitoring the quaternion data and
     * ensuring that the magnitude is always normalized to one. This
     * shouldn't happen in normal operation, but if an I2C error occurs,
     * the FIFO reads might become misaligned.
     *
     * Let's start by scaling down the quaternion data to avoid long long
     * math.
     */
    quat_q14[0] = quat[0] >> 16;
    quat_q14[1] = quat[1] >> 16;
    quat_q14[2] = quat[2] >> 16;
    quat_q14[3] = quat[3] >> 16;
    quat_mag_sq = quat_q14[0] * quat_q14[0] + quat_q14[1] * quat_q14[1] +
            quat_q14[2] * quat_q14[2] + quat_q14[3] * quat_q14[3];

#define QUAT_ERROR_THRESH       (1L<<24)
#define QUAT_MAG_SQ_NORMALIZED  (1L<<28)
#define QUAT_MAG_SQ_MIN         (QUAT_MAG_SQ_NORMALIZED - QUAT_ERROR_THRESH)
#define QUAT_MAG_SQ_MAX         (QUAT_MAG_SQ_NORMALIZED + QUAT_ERROR_THRESH)

    if ((quat_mag_sq < QUAT_MAG_SQ_MIN) || (quat_mag_sq > QUAT_MAG_SQ_MAX)) 
    {
        /* Quaternion is outside of the acceptable threshold. */
        return( DMPResetFIFO() );
    }

    /* Accelerometer */
    acc[0] = ((int16_t)fifo_data[16] << 8) | (int16_t)fifo_data[17];
    acc[1] = ((int16_t)fifo_data[18] << 8) | (int16_t)fifo_data[19];
    acc[2] = ((int16_t)fifo_data[20] << 8) | (int16_t)fifo_data[21];

    return OK;
}


/* Device-driver layer */


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
    int16_t rawAcel[3];
    int32_t rawQuat[4];
    //float a31, a32, a33;
    quaternion_t conj, r, res;
    vector_t acc, v;
    float ax, ay, az; // variables to hold latest sensor data values 
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

    retval = DMPGetRawSensor( rawAcel, rawQuat );
    if( retval != OK )
    {
        if( retval != -EAGAIN )
        {
            _err("Failed to read DMP. Release i2c\n");
            (void)stm32_i2cbus_uninitialize( i2c_bus );
            i2c_bus = NULL;
        }
        return;
    }

    /* Currently, we leave out the bias */

    /* Now we'll calculate the accleration value into actual g's */
    /*ax = (float)rawAcelTempGy[0]*A_FACTOR - fAccelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)rawAcelTempGy[1]*A_FACTOR - fAccelBias[1];   
    az = (float)rawAcelTempGy[2]*A_FACTOR - fAccelBias[2];  
    */
    ax = -((float)rawAcel[0] * A_FACTOR);  /* get actual g value, this depends on scale being set */
    ay = -((float)rawAcel[1] * A_FACTOR);   
    az = -((float)rawAcel[2] * A_FACTOR); 
    quaternion.w = (float)(rawQuat[0] >> 16) / 16384.0f;
    quaternion.x = (float)(rawQuat[1] >> 16) / 16384.0f;
    quaternion.y = (float)(rawQuat[2] >> 16) / 16384.0f;
    quaternion.z = (float)(rawQuat[3] >> 16) / 16384.0f;
   
    /* Update time period */
    now = millis();
    tick_count += (now - lastupdate);
    delta_s = (float)(now - lastupdate) * 0.001f;  /* Convert to second */
    lastupdate = now;

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
    if( tick_count >= 5000 )
    {
        if( integration_enabled < INTEGRATION_WAIT_PERIOD )
            integration_enabled++;
        tick_count = 0;
        _info( " - Quaternion: %f + %fi + %fj + %fk\n", quaternion.w, quaternion.x, quaternion.y, quaternion.z );
        _info( " - Raw accel x=%f, y=%f, z=%f\n", ax, ay, az );
        _info( " - Linear accel x=%f, y=%f, z=%f\n", accel.x, accel.y, accel.z );
        _info( " - Linear velocity x=%f, y=%f, z=%f\n", velo.x, velo.y, velo.z );
        _info( " - Linear disp x=%f, y=%f, z=%f\n", disp.x, disp.y, disp.z );
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

    ret = OK;
    switch (cmd)
    {
        case IMU_CMD_GET_SAMPLE_RATE: /* Arg: Pointer to uint32_t */
            *((uint32_t*)arg) = DMP_SAMPLE_RATE;
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
    if( DMPInit() != OK )
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
