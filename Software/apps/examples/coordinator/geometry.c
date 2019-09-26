#include <math.h>
#include <stdint.h>
#include <sys/types.h>

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

/****************************************************************************
 * Data Types
 ****************************************************************************/
typedef struct 
{
    float x;
    float y;
    float z;
}position_t;

typedef struct
{
    float w, x, y, z;
}quaternion_t;

/************************************************************************************
 * Private Data
 ************************************************************************************/
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
    q_multiply( &tmp, &v, &q_c );
    q_multiply( &q_c, q, &tmp );

    /* Copy the result */
    v->x = q_c.x;
    v->y = q_c.y;
    v->z = q_c.z;
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*-----------------------------------------------------------------------------------
 * Initialize all joint positions with respected to the machine structure.
 * This initialization already include yaw difference at the starting point.
 *----------------------------------------------------------------------------------*/
void init_positions(void)
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

    /* Rotate each known points according to the initial yaw angle */
    q_rotate_vector( &(base_pos[4]), &init_orientation );
    q_rotate_vector( &(base_pos[5]), &init_orientation );
    q_rotate_vector( &(end_pos[4]), &init_orientation );
    q_rotate_vector( &(end_pos[5]), &init_orientation );

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
}

/*-----------------------------------------------------------------------------------
 * Calculate the new position of each end-effector joints to counter 
 * the current orientation "q" (only in x and y rotation)
 *----------------------------------------------------------------------------------*/
void gen_compensate_pos( position_t* new, quaternion_t* q )
{
    quaternion_t q_diff;

    /* Calculate coutner-quaternion between q and the original one */
    q_diff.w = init_orientation.w - q->w;
    q_diff.x = -(q->x);     /* "init_orientation" has zero x and y components */
    q_diff.y = -(q->y);
    q_diff.z = 0;           /* Discard z angle */
    q_normalize( &q_diff );

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
