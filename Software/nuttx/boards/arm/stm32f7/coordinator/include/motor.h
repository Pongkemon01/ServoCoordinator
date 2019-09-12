/************************************************************************************
 * configs/coordinator/motor.h
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

#ifndef __CONFIG_BOARD_COORDINATOR_MOTOR_H
#define __CONFIG_BOARD_COORDINATOR_MOTOR_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/* Motor state */
typedef enum
{
  MOTOR_UNINIT,
  MOTOR_READY,
  MOTOR_ALARM,
  MOTOR_UNAVAILABLE,
}motor_state_t;

/* Motor commands */
#define _MIOCBASE   (_BOARDBASE | 0x0010)
#define _MIOC(id)   _IOC(_MIOCBASE, id)

#define MOTOR_CMD_INIT        _MIOC(1) /* Arg: None */
#define MOTOR_CMD_RUN         _MIOC(2) /* Arg: struct motor_run_param_t* pointer */
#define MOTOR_CMD_STOP        _MIOC(3) /* Arg: None */
#define MOTOR_CMD_CLR_ALARM   _MIOC(4) /* Arg: None */
#define MOTOR_CMD_GET_STATE   _MIOC(5) /* Arg: uint32_t* pointer */
#define MOTOR_CMD_GET_STATUS  _MIOC(6) /* Arg: uint32_t* pointer */

/* RUN command parameter
 * 
 * Motor characteristics are:
 *   - 10000 steps per revolution
 *   - Minimum speed is 5 steps per second
 *   - Maximum speed is 10000 steps per second
 */
#define MOTOR_MIN_SPEED     5
#define MOTOR_MAX_SPEED     10000
struct motor_run_param_t
{
    bool        is_cw;  /* True if truning clockwise, fasle if counter-clockwise. */
    uint16_t    speed;  /* Speed of rotation (step/second) */
    uint16_t    step;   /* Number of steps */
};

/* Public function */
int motor_initialize(void);


#endif /* __CONFIG_BOARD_COORDINATOR_MOTOR_H */
