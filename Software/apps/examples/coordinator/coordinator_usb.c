/****************************************************************************
 * examples/coordinator/coordinator_usb.c
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
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <debug.h>

/****************************************************************************
 *
 * Each token string can of the length up to 50 bytes. It must be 
 * in the format of:   "!<TOKEN>?<PARAMETERS><CR><LF>" where:
 *  - <TOKEN> is the token string of the length upto 4 characters.
 *  - <PARAMETERS> is the parameter list in the format of:
 *           <PARAMETERS> := <NAME>=<VALUE>:<PARAMETERS>
 *                          | <NAME>=<VALUE>
 *                          | nil
 *      where <NAME> and <VALUE> are strings. The total length of
 *      <PARAMETERS> must not be greater than 40 bytes.
 *  - <CR> and <LF> are the ASCII code of cariage return and line feed
 *     respectively.
 * 
 ****************************************************************************/
#define CR      '\x0D'
#define LF      '\x0A'
#define BOT     '!'     /* Begin of token mark */
#define EOT     '?'     /* End of token mark */
#define EOV     ':'     /* End of a value field */
#define EQ      '='     /* An equal sign */

/****************************************************************************
 * Type definition
 ****************************************************************************/
typedef struct
{
    char* strToken;
    char* strParam;
}s_Token_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/
static int iUsbFile = 0;
static char strTokenBuffer[60];
static s_Token_t sTokenField;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function : ParseToken
 * Parse the raw token string stored in strTokenBuffer.
 ****************************************************************************/
static void ParseToken(void)
{
    int i;

    sTokenField.strToken = NULL;
    sTokenField.strParam = NULL;
    i = 0;

    /* 1: The first byte must be '!' */
    if( strTokenBuffer[i] != BOT )
        return;
    i++;

    /* 2: Cut the Token code. The token must be less than 5 characters. */
    while( strTokenBuffer[i] != '\0' && strTokenBuffer[i] != CR && 
           strTokenBuffer[i] != LF && strTokenBuffer[i] != EOT && i < 5)
    {
        i++;
    }
    if( (i == 1) || (strTokenBuffer[i] != EOT) )
        return;     /* Token format error */
    sTokenField.strToken = &(strTokenBuffer[1]);
    strTokenBuffer[i] = '\0';   /* Cut the token part from the whole string */
    i++;

    /* 3: Cut the parameter list from the tailing */
    if( strTokenBuffer[i] == '\0' )
        return;     /* End of command so all done */
    if( (strTokenBuffer[i] == CR) || (strTokenBuffer[i] == LF) )
    {
        /* Also end of command but the tailing still exists */
        strTokenBuffer[i] = '\0';  /* Cut the tailing */
        return;
    }
    sTokenField.strParam = &(strTokenBuffer[i]);
    i++;

    /* 4: Proceed to the end of token to cut the tailing */
    while( (i < 60) && (strTokenBuffer[i] != '\0') && 
            (strTokenBuffer[i] != CR) && (strTokenBuffer[i] != LF) )
    {
        i++;
    }
    if(i == 60)
    {
        /* The token does not end within a specific length */
        sTokenField.strToken = NULL;
        sTokenField.strParam = NULL;
    }
    else
    {
        /* The token ends, then properly close the string */
        strTokenBuffer[i] = '\0';
    }
}

/****************************************************************************
 * Function : GetRawToken
 * Get a raw token string from host. It returns the length of received 
 * token string on success. Therefore, 0 means nothing to read and 
 * the value less than 0 means error in reading.
 ****************************************************************************/
static int iTokenPos = 0;  /* Position of current character in token string */
static int GetRawToken(void)
{
    char    c;
    int     retval;

    //DEBUGASSERT(iUsbFile > 0);

    for( ;iTokenPos < 60;iTokenPos++ )
    {
        retval = read( iUsbFile, &c, 1 );   /* Fetch 1 byte */
        if( retval <= 0 )
        {
            if( retval < 0 && errno != EWOULDBLOCK )
            {
                /* Error, void all previously received token */
                iTokenPos = 0;
                return retval;
            }
            else
            {
                /* Nothing to read but the token string has not completed */
                return 0;
            }
        }

        /* Got a valid character */
        if( ( c == CR || c == LF ) )
        {
            /* If either CR or LF is the first received character, 
             * we simply discard it and continue with the next reading.
             * However, if it is not the fist character, it means
             * the token is closing. */
            if( iTokenPos > 0 )
            {
                /* Closing the token and return the newly got token */
                strTokenBuffer[ iTokenPos ] = '\0';
                retval = iTokenPos;
                iTokenPos = 0;  /* Clear state back to the beginning */
                return retval;
            }
        }
        else
        {
            /* Got a normal character. Just store it. */
            strTokenBuffer[ iTokenPos ] = c;
        }
    }

    /* Reaching here means that we proceed to the position 60.
     * Therefore, we must discarded all received buffer content. */
    iTokenPos = 0;
    return 0;
}

/****************************************************************************
 * Function : GetNextParam
 * Get a raw token string from host. It returns the length of received 
 * token string on success. Therefore, 0 means nothing to read and 
 * the value less than 0 means error in reading.
 ****************************************************************************/
static char* GetNextParam( char* pcStartPos )
{
    char *t;

    t = pcStartPos;

    /* Seek to the next value separator ':' or end-of-sgring */
    while( ( *t != EOV ) && ( *t != '\0') )
        t++;

    /* If current position is ':', then advance to the next character */
    if( *t == EOV )
        t++;

    /* If no further parameter, return NULL */
    if( *t == '\0' )
        return NULL;
    else
        return t;
}

/****************************************************************************
 * Function : GetValue
 * Copy the parameter value starting at strStart to the buffer strDest with
 * the maximum length of iMexLen. The function returns the number of the
 * characters copied.
 ****************************************************************************/
static int GetValue( char* strStart, char* strDest, int iMaxLen )
{
    int i;

    for(i = 0; i < iMaxLen; i++ )
    {
        if( ( *strStart == EOV ) || ( *strStart == '\0' ) )
            break;  /* Reaching the end of value */
        *strDest = *strStart;
        strDest++;
        strStart++;
    }

    /* Close the destination string */
    *strDest = '\0';

    return i;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function : GetParamFromKey
 * Extract the parameter value, as a string, associated to the provided key.
 * The function returns the length of gotten value (0 means the sprcified
 * key is not found).
 ****************************************************************************/

int GetParamFromKey( char* strKey, char* strValue, int iMaxLen )
{
    char* s;
    char* t;

    *strValue = '\0';

    /* Starting from the fist parameter */
    t = sTokenField.strParam;

    while( t != NULL )
    {
        s = strKey;
        while( *t == *s )
        {
            s++;
            t++;
        }
        /* If the first unmatched characteris '=' at *t and '\0' at *s,
        * it means that the key is found. */
        if( ( *s == '\0' ) && ( *t == EQ ) )
        {
            /* Matched */
            t++;    /* Skip '=' */
            /* Copy the value */
            return ( GetValue( t, strValue, iMaxLen ) );
        }
        else
        {
            /* Current parameter is unmatched */
            t = GetNextParam( t );

            if( t == NULL )
                return 0;   /* Not found at all */
        }
    }

    /* Reaching here means t == NULL */
    return 0;
}

/****************************************************************************
 * Function : GetNextToken
 * Receive the next token from USB communication. It returns the token string
 * on success. Otherwise, NULL is returned.
 ****************************************************************************/
char* GetNextToken(void)
{
    if( iUsbFile <= 0 )
        return NULL;    /* Return nothing if the USB is unavailable */

    if( GetRawToken() <= 0 )
        return NULL;    /* Return nothing if USB does not provide new token */

    ParseToken();        /* Split raw token into token and parameters */

    return( sTokenField.strToken ); /* Return the token string */
}

/****************************************************************************
 * Function : SendResponse
 * Helper function to write a string to USB. It returns the value from 
 * standard file "write" command.
 ****************************************************************************/
int SendResponse( char* strResponse, int iMaxLen )
{
    if( iUsbFile <= 0 )
        return 0;       /* Return nothing if the USB is unavailable */

    if( iMaxLen <= 0 )
        return 0;       /* Nothing to write */

    return( write( iUsbFile, strResponse, iMaxLen ) );
}

/****************************************************************************
 * Function : InitUSB
 * Initialize the USB connection. It returns the value from "open" call.
 * The returned value is actually the file description number. Hence, 
 * something less than or equal to 0 means error.
 ****************************************************************************/
int InitUSB(void)
{
    iUsbFile = open( "/dev/ttyACM0", ( O_NONBLOCK | O_RDWR ) );

    if(iUsbFile <= 0)
        _err("Cannot open USB connection\n");
    return iUsbFile;
}
