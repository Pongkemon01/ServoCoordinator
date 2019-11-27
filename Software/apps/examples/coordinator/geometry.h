#ifndef __COORDINATOR_GEOMETRY_H
#define __COORDINATOR_GEOMETRY_H

/****************************************************************************
 * Data Types
 ****************************************************************************/
typedef struct 
{
    float x, y, z;
}vector_t;

typedef struct
{
    float w, x, y, z;
}quaternion_t;

typedef float rot_matrix_t[3][3];

void init_geometry( void );
void vector_rotate_m( vector_t* v, rot_matrix_t m );
void shift_to_motor_frame( vector_t* newpos, vector_t* pos, uint32_t pos_index );
void get_init_ee_pos( vector_t out[] );
float inverse_kinematic( vector_t *pos, bool is_first_angle );
#endif
