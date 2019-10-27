#ifndef __COORDINATOR_CONTROL_H
#define __COORDINATOR_CONTROL_H

/* Control constants */
#define KP  5.0f
#define KI  0.2f
#define KD  0.2f

#define LPF_OMEGA 10.0f     /* Starts from 10 rad/s */

#define INTEGRATOR_MAX  150.0f

typedef struct
{
    /* Integration term */
    float prev_err;     /* Previous error */
    float inte_acc;     /* Accumulator for integration */

    /* Derivative term */
    float prev_out;     /* Previous output */
    float prev_d_err;   /* Previous derivative input */
    float prev_d_out;   /* Previous derivative output */

    /* General state */
    uint32_t prev_t;    /* Previous iteration time */
}PID_State_t;

float PID( float set_point, float curent, PID_State_t* pid);
void InitState(PID_State_t* s);

#endif
