#include <math.h>

typedef struct
{
    float w, x, y, z;
}quaternion_t;

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

void q_normalize( quaternion_t* q )
{
    float invnorm;

    invnorm = InvSqrt( (q->w * q->w ) + ( q->x * q->x ) + 
                     ( q->y * q->y ) + ( q->z * q->z ) );
    q->w *= invnorm;
    q->x *= invnorm;
    q->y *= invnorm;
    q->z *= invnorm;
}

void q_multiply( quaternion_t* res, quaternion_t* q1, quaternion_t* q2 )
{
    res->w = (q1->w*q2->w) - (q1->x*q2->x) - (q1->y*q2->y) - (q1->z*q2->z);
    res->x = (q1->w*q2->x) + (q1->x*q2->w) + (q1->y*q2->z) - (q1->z*q2->y);
    res->y = (q1->w*q2->y) - (q1->x*q2->z) + (q1->y*q2->w) + (q1->z*q2->x);
    res->z = (q1->w*q2->z) + (q1->x*q2->y) - (q1->y*q2->x) + (q1->z*q2->w);
}

void q_conjugate( quaternion_t* q )
{
    q->x = -(q->x);
    q->y = -(q->y);
    q->z = -(q->z);
}

/*-----------------------------------------------------------------------------------
 * Create a quaternion with just angle around Z-axis from the given quaternion
 *----------------------------------------------------------------------------------*/
void q_get_z( quaternion_t* res, quaternion_t* q )
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

/* Calculate relative quaternion r from q1 to q2 ( q2 = r.q1 => q2.q1* = r ) */
void q_transformback_q( quaternion_t* res, quaternion_t* q1, quaternion_t* q2 )
{
    quaternion_t t;

    /* Conjugate q1 */
    t->w = q1->w;
    t->x = -(q1->x);
    t->y = -(q1->y);
    t->z = -(q1->z);

    /* multiply q2 with q1* to get the result */
    q_multiply( res, q2, &t );
}

/*-----------------------------------------------------------------------------------
 * https://dreamanddead.github.io/2019/04/24/understanding-euler-angles.html
 *----------------------------------------------------------------------------------*/
