#ifndef __COORDINATOR_GEOMETRY_H
#define __COORDINATOR_GEOMETRY_H

/****************************************************************************
 * Data Types
 ****************************************************************************/
typedef struct 
{
    float x, y, z;
}position_t;

typedef struct
{
    float w, x, y, z;
}quaternion_t;

void init_geometry( quaternion_t *start_orientation );
void gen_compensate_pos( position_t new[], quaternion_t* q, position_t* p );
float inverse_kinematic( position_t *pos );
#endif
