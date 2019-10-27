#include <math.h>
#include <stdint.h>
#include <sys/types.h>

#include "geometry.h"

/****************************************************************************
 * There are 3 reference frames in this system.
 *  1. The fixed world frame with x-axis points to the Earth magnetic
 *     north, y-axis points to the east, and z-axis points down.
 *  2. The platform frame having the origin at the center of
 *     platform-base with x-axis points forward and z-axis points down.
 *  3. The end-effector frame having the origin at the center of
 *     the end-effector with x-axis points toward the IMU sensor
 *     and z-axis points down.
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define F_PI        3.1415926535897932384626433832795029f
#define DEG_TO_RAD  ( F_PI / 180.0f )
#define RAD_TO_DEG  ( 180.0f / F_PI )

#define JOINT_P_ANGLE       (120.0f* DEG_TO_RAD)
#define BASE_RADIUS         (220.0f) /* Radius from center of base to each joint */
#define END_RADIUS          (160.0f) /* Radius from center of end-effector to each joint */
#define BASE_JOINT_SPACE    (256.0f) /* Space between base joints on the same side */
#define END_JOINT_SPACE     (45.0f)  /* Space between end-effector joints on the same side */
#define BISEP_LENGTH        (95.0f)  /* Length of an actuated arm */
#define FORE_LENGTH         (110.0f) /* Length of a free arm */

/************************************************************************************
 * Private Data
 ************************************************************************************/
/* Coordinate of IMU sensor respected to the origin of the end-effector. */
const position_t sensor_disp = 
{
    .x = 160.0f,
    .y = 0,
    .z = 0,
};

/* Coordinate displacement between base frame and end-effector frame */
const position_t end_effector_disp =
{
    .x = 0,
    .y = 0,
    .z = -100.0f,
};

static position_t base_pos[6], end_pos[6];
static quaternion_t init_orientation;

/************************************************************************************
 * Private Functions
 ************************************************************************************/
/*-----------------------------------------------------------------------------------
 * Quaternion math
 *----------------------------------------------------------------------------------*/
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

static void q_normalize( quaternion_t* q )
{
    float invnorm;

    invnorm = InvSqrt( (q->w * q->w ) + ( q->x * q->x ) + 
                     ( q->y * q->y ) + ( q->z * q->z ) );
    q->w *= invnorm;
    q->x *= invnorm;
    q->y *= invnorm;
    q->z *= invnorm;
}

static void q_multiply( quaternion_t* res, quaternion_t* q1, quaternion_t* q2 )
{
    res->w = (q1->w*q2->w) - (q1->x*q2->x) - (q1->y*q2->y) - (q1->z*q2->z);
    res->x = (q1->w*q2->x) + (q1->x*q2->w) + (q1->y*q2->z) - (q1->z*q2->y);
    res->y = (q1->w*q2->y) - (q1->x*q2->z) + (q1->y*q2->w) + (q1->z*q2->x);
    res->z = (q1->w*q2->z) + (q1->x*q2->y) - (q1->y*q2->x) + (q1->z*q2->w);
}

/* Calculate anti quaternion to make "qc" orientation becomes "qr" orientation */
static void q_anti_relative( quaternion_t* res, quaternion_t* qc, quaternion_t* qr )
{
    quaternion_t qc_c;

    /* qc_c = conjugate of qc */
    qc_c.w = qc->w;
    qc_c.x = -(qc->x);
    qc_c.y = -(qc->y);
    qc_c.z = -(qc->z);

    q_multiply( res, qr, &qc_c );
}

/*-----------------------------------------------------------------------------------
 * Create a quaternion with just angle around Z-axis from the given quaternion
 *----------------------------------------------------------------------------------*/
static void q_get_z( quaternion_t* res, quaternion_t* q )
{
    float inv_norm;

    /* copy only y angle */
    res->w = q->w;
    res->z = q->z;
    /* Zero all other angles */
    res->x = 0;
    res->y = 0;

    /* Normalize the result ( Use custom calculation instead of calling
       the normalizaion function because some terms are known to be zero,
       which can be optimized here. */
    inv_norm = InvSqrt( ( res->w * res->w ) + ( res->z * res->z ) );
    res->w *= inv_norm;
    res->z *= inv_norm;
}

/*-----------------------------------------------------------------------------------
 * Rotate vector "v" with quaternion "q" and give the result back into "v"
 *----------------------------------------------------------------------------------*/
static void q_rotate_vector( position_t* v, quaternion_t* q )
{
    quaternion_t q_c, qv, tmp;

    /* Conjugate q */
    q_c.w = q->w;
    q_c.x = -( q->x );
    q_c.y = -( q->y );
    q_c.z = -( q->z );

    /* Transfor input into quaternion */
    qv.w = 0.0f;
    qv.x = v->x;
    qv.y = v->y;
    qv.z = v->z;

    /* Rotate and store result in q_c */
    q_multiply( &tmp, &qv, &q_c );
    q_multiply( &qv, q, &tmp );

    /* Copy the result */
    v->x = qv.x;
    v->y = qv.y;
    v->z = qv.z;
}

/*-----------------------------------------------------------------------------------
 * End of Quaternion math
 *----------------------------------------------------------------------------------*/

/* Rotate vector v around axis z with the specified angle */
static void vector_rotate_z( position_t* v, float angle )
{
    float s, c;
    float x, y;

    s = sinf( angle );
    c = cosf( angle );

    x = (c * v->x) - (s * v->y);
    y = (s * v->x) + (c * v->y);

    v->x = x;
    v->y = y;
}

/*-----------------------------------------------------------------------------------
 * Initialize all joint positions with respected to the machine structure.
 * This initialization already include yaw difference at the starting point.
 *----------------------------------------------------------------------------------*/
static void init_positions(void)
{
    /* Initialize some known positions */
    base_pos[4].x = -( BASE_RADIUS * cosf( asinf( ( BASE_JOINT_SPACE / 2.0f ) / BASE_RADIUS) ) );
    base_pos[4].y = BASE_JOINT_SPACE / 2.0f;
    base_pos[4].z = 0;
    base_pos[5].x = base_pos[4].x;
    base_pos[5].y = -(base_pos[4].y);
    base_pos[5].z = 0;

    end_pos[4].x = -( END_RADIUS * cosf( asinf( ( END_JOINT_SPACE / 2.0 ) / END_RADIUS ) ) );
    end_pos[4].y = END_JOINT_SPACE / 2.0;
    end_pos[4].z = 0;
    end_pos[5].x = end_pos[4].x;
    end_pos[5].y = -(end_pos[4].y);
    end_pos[5].z = 0;

    /* Copy coordinates of the known points to relavant points */
    base_pos[0].x = base_pos[4].x;
    base_pos[0].y = base_pos[4].y;
    base_pos[0].z = base_pos[4].z;
    base_pos[1].x = base_pos[5].x;
    base_pos[1].y = base_pos[5].y;
    base_pos[1].z = base_pos[5].z;

    base_pos[2].x = base_pos[4].x;
    base_pos[2].y = base_pos[4].y;
    base_pos[2].z = base_pos[4].z;
    base_pos[3].x = base_pos[5].x;
    base_pos[3].y = base_pos[5].y;
    base_pos[3].z = base_pos[5].z;

    end_pos[0].x = end_pos[4].x;
    end_pos[0].y = end_pos[4].y;
    end_pos[0].z = end_pos[4].z;
    end_pos[1].x = end_pos[5].x;
    end_pos[1].y = end_pos[5].y;
    end_pos[1].z = end_pos[5].z;

    end_pos[2].x = end_pos[4].x;
    end_pos[2].y = end_pos[4].y;
    end_pos[2].z = end_pos[4].z;
    end_pos[3].x = end_pos[5].x;
    end_pos[3].y = end_pos[5].y;
    end_pos[3].z = end_pos[5].z;

    /* Rotate the copied coordinates to their real coordinates */
    vector_rotate_z( &(base_pos[0]), JOINT_P_ANGLE );
    vector_rotate_z( &(base_pos[1]), JOINT_P_ANGLE );
    vector_rotate_z( &(base_pos[2]), -JOINT_P_ANGLE );
    vector_rotate_z( &(base_pos[3]), -JOINT_P_ANGLE );
    vector_rotate_z( &(end_pos[0]), JOINT_P_ANGLE );
    vector_rotate_z( &(end_pos[1]), JOINT_P_ANGLE );
    vector_rotate_z( &(end_pos[2]), -JOINT_P_ANGLE );
    vector_rotate_z( &(end_pos[3]), -JOINT_P_ANGLE );

    /* We don't have to shift the end-effector origin to IMU position
     * because the end-effector is a rigid body. All points on this 
     * body would have same orientation as well as linear movements.
     */

    /* Finally, adjust each points according to the initial orientation */
    for(int i = 0; i < 6; i++)
    {
        q_rotate_vector( &(base_pos[i]), &init_orientation );
        q_rotate_vector( &(end_pos[i]), &init_orientation );
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*-----------------------------------------------------------------------------------
 * Initialize system geometry.
 *----------------------------------------------------------------------------------*/
void init_geometry( quaternion_t *start_orientation )
{
    float   yaw;
    quaternion_t *q = start_orientation; /* Just for shorter name */

    /* Initialize the orientation of the origin with the current yaw */

    /* Extracting Yaw by the formula from 
    https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles */
    yaw = atan2f( 2.0f * (q->w * q->z + q->x * q->y), 
                1.0f - (2.0f * (q->y * q->y + q->z * q->z)) );
    
    /* Convert yaw to quaternion */
    yaw *= 0.5f;
    init_orientation.w = cosf(yaw);
    init_orientation.x = 0.0f;
    init_orientation.y = 0.0f;
    init_orientation.z = sinf(yaw);

    /* Then assign all joint positions according to the design and the orientation */
    init_positions();
}

/*-----------------------------------------------------------------------------------
 * Calculate the new position of each end-effector joints to counter 
 * the current orientation "q" and position "p"
 *----------------------------------------------------------------------------------*/
void gen_compensate_pos( position_t new[], quaternion_t* q, position_t* p )
{
    quaternion_t q_diff;

    /* Calculate coutner-quaternion between q and the original one */
    q_anti_relative( &q_diff, q, &init_orientation );

    /* Initialize result coordinates with initial coordinates then rotate
        those coordinates with respected to the difference quaternion. */
    for( int i = 0; i <= 6; i++ )
    {
        new[i].x = end_pos[i].x;
        new[i].y = end_pos[i].y;
        new[i].y = end_pos[i].y;
        q_rotate_vector( &(new[i]), &q_diff );
    }
}

/*-----------------------------------------------------------------------------------
 * Calculate inverse kinematic of a given end-effector joint position
 * to motor angle.
 *----------------------------------------------------------------------------------*/
float inverse_kinematic( position_t *pos )
{
    float theta1;
    float cos_theta3;
    float sin_theta2, cos_theta2;
    float fa_cos, baba_fa_cos;      /* Temporary variables */

    cos_theta3 = cosf( asinf( pos->y / FORE_LENGTH ) );

    fa_cos = FORE_LENGTH * cos_theta3;
    baba_fa_cos = ( BISEP_LENGTH * BISEP_LENGTH ) + ( fa_cos * fa_cos );
    cos_theta2 = ( pos->x * pos->x ) + ( pos->z * pos->z ) - baba_fa_cos;
    cos_theta2 /= ( 2.0f * BISEP_LENGTH * fa_cos );

    sin_theta2 = FastSqrt( 1 - (cos_theta2 * cos_theta2) ); /* Sine value can both + and - */

    theta1 = atan2f( pos->z, pos->x ) - atan2f( (FORE_LENGTH * cos_theta3 * sin_theta2), (BISEP_LENGTH + (cos_theta2 * fa_cos)) );

    return( theta1 );
}
