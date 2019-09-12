/****************************************************************************
 * examples/coordinator/coordinator_token_parser.c
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

#include <string.h>

#include "coordinator_token_parser.h"
#include "coordinator_actor.h"

struct s_token_t
{
    char        strToken[6];
    token_op_t  eToken;
};

static struct s_token_t s_token_table[] =
{
    { .strToken = "RES", .eToken = DoReboot },
    { .strToken = "GAM", .eToken = DoGetAvailableMotor },
    { .strToken = "INM", .eToken = DoInitMotor },
    { .strToken = "GMS", .eToken = DoGetMotorStatus },
    { .strToken = "CMA", .eToken = DoClearMotorAlarm },
    { .strToken = "STP", .eToken = DoStopMotor },
    { .strToken = "RUN", .eToken = DoRunMotor },
    { .strToken = "GQE", .eToken = DoGetQEValue },
    { .strToken = "RQE", .eToken = DoResQEValue },
    { .strToken = "HME", .eToken = DoHomeValue },

    /* The special token to mark the closing of token table */
    { .strToken = ".",   .eToken = DoUnknown }
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function : GetTokenValue
 * Parse the strToken and return the value associated to that token.
 ****************************************************************************/

token_op_t GetTokenValue( char* strToken )
{
    for( int i = 0;i < 60; i++ ) /* It can be counted to any number greater
                                    than the number of token. 60 is just 
                                    picked here */
    {
        if( s_token_table[i].strToken[0] == '.' )
            break;  /* Hitting the guard item. Then break out the loop */

        if( strcmp( strToken, s_token_table[i].strToken ) == 0 )
            return ( s_token_table[i].eToken );
    }

    return DoUnknown;
}
