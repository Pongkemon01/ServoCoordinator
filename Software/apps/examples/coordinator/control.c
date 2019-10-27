#include <math.h>
#include <stdint.h>
#include <sys/types.h>
#include <time.h>

#include "control.h"

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

    return ( ( ( (uint32_t)tp.tv_sec ) * 1000 ) + ( tp.tv_nsec / 1000000 ) );
}

/************************************************************************************
 * Pubic Functions
 ************************************************************************************/
void InitState(PID_State_t* s)
{
    s->prev_err = 0;
    s->inte_acc = 0;
    s->prev_out = 0;
    s->prev_d_err = 0;
    s->prev_d_out = 0;
    s->prev_t = millis();
}

/************************************************************************************
 * The discretization follows Bilinear method (s = (2/T)*((z-1)/(z+1))). The term
 * P, I, and D canbe calculated separately and compbined at the final.
 * Some modifications are made to derivative term to reduce noise but retain
 * system damping as:
 * 
 * - To avoid that changes in the desired process value makes any unwanted
 * rapid changes in the control input, the controller is improved by basing 
 * the derivative term on the process value (output) only.
 * 
 * - A first-order low-pass filter is applied to the derivative term to reduce
 * transcient.
 ************************************************************************************/
float PID( float set_point, float current, PID_State_t* pid)
{
    float e, ed;
    float p, i, d;
    float h_delta_s;    /* Half of sampling period */
    float d_temp;       /* Temporary for derivative term */

    uint32_t current_time;

    current_time = millis();
    h_delta_s = (float)(current_time - pid->prev_t) * 0.5f;

    e = set_point - current;
    ed = pid->prev_out - current;   /* Take error from only output change */

    /* Calculate P */
    p = KP * e;

    /* Calculate I */
    i = ( KI * h_delta_s * (e + pid->prev_err) ) + pid->inte_acc;
    if( i > INTEGRATOR_MAX )        /* Integral wind-up limit */
        i = INTEGRATOR_MAX;
    else if( i < -INTEGRATOR_MAX )
        i = -INTEGRATOR_MAX;

    /* Calculate D */
    d_temp = LPF_OMEGA * h_delta_s;
    d = (KD * LPF_OMEGA * ( ed + pid->prev_d_err ) ) - ( (1.0 - d_temp) * pid->prev_d_out );
    d /= ( 1.0f + d_temp );

    /* Store data back to the state structure */
    pid->prev_t = current_time;
    pid->prev_err = e;
    pid->inte_acc = i;
    pid->prev_d_err = ed;
    pid->prev_d_out = d;
    pid->prev_out = p + i + d;

    return( pid->prev_out );
}
