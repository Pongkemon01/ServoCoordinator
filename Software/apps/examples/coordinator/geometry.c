#include <debug.h>
#include <assert.h>

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
#define COS_30      0.866f
#define F_PI        3.1415926535897932384626433832795029f
#define F_PI_2      (F_PI / 2.0f)
#define DEG_TO_RAD  ( F_PI / 180.0f )
#define RAD_TO_DEG  ( 180.0f / F_PI )

#define JOINT_P_ANGLE       (120.0f* DEG_TO_RAD)
#define BASE_RADIUS         (195.0f) /* Radius from center of base to each joint */
#define END_RADIUS          (160.0f) /* Radius from center of end-effector to each joint */
#define BASE_JOINT_SPACE    (256.0f) /* Space between base joints on the same side */
#define END_JOINT_SPACE     (45.0f)  /* Space between end-effector joints on the same side */
#define BISEP_LENGTH        (95.0f)  /* Length of an actuated arm */
#define FORE_LENGTH         (400.0f) /* Length of a free arm */
/* Initial displacement between base frame and end-effector frame where angle = 0 */
#define END_DISP            (-345.9365f)

/************************************************************************************
 * Private Data
 ************************************************************************************/
static vector_t base_pos[6], end_pos[6];

/*
 * Rotation matrix for joint transformation to motor frame.
 * These matrices were verified bt python code. They are:
 * r_matrix_j1 = np.array(((-cos_30, -0.5, 0),(-0.5, cos_30, 0),(0, 0, -1)))
 * r_matrix_j3 = np.array(((cos_30, -0.5, 0), (-0.5, -cos_30, 0), (0, 0, -1)))
 * r_matrix_j5 = np.array(((0, 1, 0), (1, 0, 0), (0, 0, -1)))
 */
static rot_matrix_t rot_m_j1j2 = 
{
    { -COS_30, -0.5f,   0    },
    { -0.5f,    COS_30, 0    },
    { 0,        0,     -1.0f } 
};

static rot_matrix_t rot_m_j3j4 = 
{
    { COS_30,  -0.5f,   0    },
    { -0.5f,   -COS_30, 0    },
    { 0,       0,      -1.0f } 
};

static rot_matrix_t rot_m_j5j6 = 
{
    { 0,     1.0f,  0     },
    { 1.0f,  0,     0     },
    { 0,     0,     -1.0f }
};

/************************************************************************************
 * Private Functions
 ************************************************************************************/
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

/*-----------------------------------------------------------------------------------
 * Vector math
 *----------------------------------------------------------------------------------*/
/* Rotate vector v around axis z with the specified angle */
static void vector_rotate_z( vector_t* v, float angle )
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

/* Generate gravitational-alignment rotation matrix.
 * The algorith came from matlab function
 * function R=RotationFromTwoVectors(A, B)
 *  v = cross(A,B);
 *  ssc = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
 *  R = eye(3) + ssc + ssc^2*(1-dot(A,B))/(norm(v))^2;
 * 
 * The function is optimized on the fact that B is always [0, 0, 1]
 * 
 */
// void gen_anti_rotation( rot_matrix_t out, vector_t* g )
// {
//     float f, nv;

//     /* v = cross(g, [0, 0, 1])
//      * v = [g->y, g->x, 0]
//      */
//     /* nv = (norm(v))^2; Eucledian norm = sqrt(x^2 + y^2) so norm^2 = x^2 + y^2 */
//     nv = (g->x * g->x) + (g->y * g->y);
    
//     /* f = (1-dot(A,B))/(norm(v))^2 */
//     if( nv == 0)
//         f = 0.0f;
//     else
//         f = (1.0f - g->z) / nv;

//     /* out = eye(3) + ssc + f*ssc^2 */
//     out[0][0] = 1.0f - (f * g->x * g->x);
//     out[0][1] = (f * g->x * g->y);
//     out[0][2] = g->x;
//     out[1][0] = out[0][1] ;  /* (f * g->x * g->y) */
//     out[1][1] = 1.0f - (f * g->y * g->y);
//     out[1][2] = -(g->y);
//     out[2][0] = -(g->x);
//     out[2][1] = g->y;
//     out[2][2] = 1.0f - (f * ((g->x * g->x) + (g->y * g->y)));
// }

/* Rotate vector v by the given rotation matrix */
void vector_rotate_m( vector_t* v, rot_matrix_t m )
{
    float nx, ny, nz;

    nx = (m[0][0] * v->x) + (m[0][1] * v->y) + (m[0][2] * v->z);
    ny = (m[1][0] * v->x) + (m[1][1] * v->y) + (m[1][2] * v->z);
    nz = (m[2][0] * v->x) + (m[2][1] * v->y) + (m[2][2] * v->z);

    v->x = nx;
    v->y = ny;
    v->z = nz;
}

/*-----------------------------------------------------------------------------------
 * End of Vector math
 *----------------------------------------------------------------------------------*/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*-----------------------------------------------------------------------------------
 * Initialize all joint positions with respected to the machine structure.
 * This initialization already include yaw difference at the starting point.
 * Therefore, all coordinates have the same origin at the center of the base.
 *----------------------------------------------------------------------------------*/
void init_geometry( void )
{
    /* Initialize some known positions */
    base_pos[4].x = -( BASE_RADIUS * cosf( asinf( ( BASE_JOINT_SPACE / 2.0f ) / BASE_RADIUS) ) );
    base_pos[4].y = BASE_JOINT_SPACE / 2.0f;
    base_pos[4].z = 0;
    base_pos[5].x = base_pos[4].x;
    base_pos[5].y = -(base_pos[4].y);
    base_pos[5].z = 0;

    end_pos[4].x = -( END_RADIUS * cosf( asinf( ( END_JOINT_SPACE / 2.0f ) / END_RADIUS ) ) );
    end_pos[4].y = END_JOINT_SPACE / 2.0f;
    end_pos[4].z = END_DISP;
    end_pos[5].x = end_pos[4].x;
    end_pos[5].y = -(end_pos[4].y);
    end_pos[5].z = END_DISP;

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
}

/*-----------------------------------------------------------------------------------
 * Update all end-effector joint positions.
 *----------------------------------------------------------------------------------*/
void get_init_ee_pos( vector_t out[] )
{
    for(int i = 0; i < 6; i++ )
    {
        out[i].x = end_pos[i].x;
        out[i].y = end_pos[i].y;
        out[i].z = end_pos[i].z;
    }
}

/*-----------------------------------------------------------------------------------
 * Shift a given global position from base-frame origin to motor origin
 * and rotate the frame to make the corresponding bisep-arm moves along 
 * XZ plane, where the XYZ plane follows NWU norm (instead of NED as usual).
 *----------------------------------------------------------------------------------*/
void shift_to_motor_frame( vector_t* newpos, vector_t* pos, uint32_t pos_index )
{
    DEBUGASSERT( pos_index < 6 );

    /* Shifting the frame */
    newpos->x = pos->x - base_pos[pos_index].x;
    newpos->y = pos->y - base_pos[pos_index].y;
    newpos->z = pos->z - base_pos[pos_index].z;

    /* Rotate the frame */
    switch( pos_index )
    {
        case 0:
        case 1:
            vector_rotate_m( newpos, rot_m_j1j2 );
            break;
        case 2:
        case 3:
            vector_rotate_m( newpos, rot_m_j3j4 );
            break;
        case 4:
        case 5:
            vector_rotate_m( newpos, rot_m_j5j6 );
            break;
    }
    // _info("Pre joint %d  POS=[%f|%f|%f] base=[%f|%f|%f] New=[%f|%f|%f]\n", 
    // pos_index, pos->x, pos->y, pos->z, base_pos[pos_index].x, base_pos[pos_index].y, base_pos[pos_index].z,
    // newpos->x, newpos->y, newpos->z);
}

/*-----------------------------------------------------------------------------------
 * Calculate inverse kinematic of a given end-effector joint position
 * to motor angle.
 *----------------------------------------------------------------------------------*/
float inverse_kinematic( vector_t *pos, bool is_first_angle )
{
    float theta;
    float f_dat, gamma, cos_phi;

    f_dat = FORE_LENGTH * cosf( asinf( pos->y / FORE_LENGTH ) );
    gamma = atan2f( pos->z, pos->x );
    cos_phi = ( pos->x * pos->x ) + ( pos->z * pos->z ) + ( BISEP_LENGTH * BISEP_LENGTH ) - ( f_dat * f_dat );
    cos_phi /= ( 2.0f * BISEP_LENGTH * FastSqrt( ( pos->x * pos->x ) + ( pos->z * pos->z ) ) );
    if( is_first_angle )
        theta = gamma - acosf( cos_phi );
    else
        theta = F_PI - (gamma + acosf( cos_phi ));

    return( theta );
}

float inverse_kinematic_2( vector_t* pos, bool is_first_angle )
{
    float cos_theta3, cos_theta2, sin_theta2, theta1;
    float fa_cos;

    cos_theta3 = cosf( asinf( pos->y / FORE_LENGTH ) );
    fa_cos = FORE_LENGTH * cos_theta3;      /* Projection of the forearm on x-z plane */

    cos_theta2 = ( pos->x * pos->x ) + ( pos->z * pos->z ) - (( BISEP_LENGTH * BISEP_LENGTH ) + ( fa_cos * fa_cos ));
    cos_theta2 /= ( 2.0f * BISEP_LENGTH * fa_cos );

    /*  Sine value can both + and - */
    sin_theta2 = FastSqrt( 1.0f - (cos_theta2 * cos_theta2) );
    if( !is_first_angle )
        sin_theta2 = -sin_theta2;
    // sin_theta2_2 = -sin_theta2

    theta1 = atan2f( pos->z, pos->x ) - atan2f( (FORE_LENGTH * cos_theta3 * sin_theta2), (BISEP_LENGTH + (cos_theta2 * fa_cos)) );
    if( !is_first_angle )
        theta1 = F_PI - theta1;

    return( theta1 );
}
