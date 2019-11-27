/************************************************************************************
 * configs/coordinator/imu.h
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

#ifndef __CONFIG_BOARD_COORDINATOR_IMU_H
#define __CONFIG_BOARD_COORDINATOR_IMU_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/* I2C channel use to connect to the IMU */
#define I2C_CHANNEL             2

/* Motor commands */
#define _IIOCBASE   (_BOARDBASE | 0x0020)
#define _IIOC(id)   _IOC(_IIOCBASE, id)

#define IMU_CMD_GET_SAMPLE_RATE _IIOC(1) /* Arg: Pointer to uint32_t */
#define IMU_CMD_GET_QUATERNION  _IIOC(2) /* Arg: Array float[4] */
#define IMU_CMD_GET_GRAVITY     _IIOC(3) /* Arg: Array float[3] for x/y/z */
#define IMU_CMD_GET_LIN_ACCEL   _IIOC(4) /* Arg: Array float[3] for x/y/z */
#define IMU_CMD_GET_LIN_VELO    _IIOC(5) /* Arg: Array float[3] for x/y/z */
#define IMU_CMD_GET_LIN_DISP    _IIOC(6) /* Arg: Array float[3] for x/y/z */
#define IMU_CMD_GET_ANG_VELO    _IIOC(7) /* Arg: Array float[3] for x/y/z */
#define IMU_CMD_GET_ANG_DISP    _IIOC(8) /* Arg: Array float[3] for x/y/z */
#define IMU_CMD_SET_FILTER_COEF _IIOC(9) /* Arg: Pointer to float */
#define IMU_CMD_GET_TILT        _IIOC(10) /* Arg: Array float[3] for x/y/z */

/* Public function */
int imu_initialize(void);


#endif /* __CONFIG_BOARD_COORDINATOR_IMU_H */
