import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import math
from scipy import signal

BISEP_LENGTH = 95.0
FORE_LENGTH = 400.0

def inverse_kinematic( x, y, z ):
    f_dat = FORE_LENGTH * math.cos( math.asin( y / FORE_LENGTH ) )
    theta_phi = math.atan2( z, x )
    cos_phi = (x * x) + (z * z) + (BISEP_LENGTH * BISEP_LENGTH) - (f_dat * f_dat)
    cos_phi /= ( 2.0 * BISEP_LENGTH * math.sqrt( (x * x) + (z * z) ) )
    phi = math.acos( cos_phi )
    return (theta_phi - phi), math.pi - (theta_phi + phi)

def inverse_kinematic_2( x, y, z ):
    cos_theta3 = math.cos( math.asin( y / FORE_LENGTH ) )

    fa_cos = FORE_LENGTH * cos_theta3
    baba_fa_cos = ( BISEP_LENGTH * BISEP_LENGTH ) + ( fa_cos * fa_cos )
    cos_theta2 = ( x * x ) + ( z * z ) - baba_fa_cos
    cos_theta2 /= ( 2.0 * BISEP_LENGTH * fa_cos )

    #  Sine value can both + and -
    sin_theta2_1 = math.sqrt( 1.0 - (cos_theta2 * cos_theta2) )
    sin_theta2_2 = -sin_theta2_1

    theta1 = math.atan2( z, x ) - math.atan2( (FORE_LENGTH * cos_theta3 * sin_theta2_1), (BISEP_LENGTH + (cos_theta2 * fa_cos)) )
    theta2 = math.atan2( z, x ) - math.atan2( (FORE_LENGTH * cos_theta3 * sin_theta2_2), (BISEP_LENGTH + (cos_theta2 * fa_cos)) )

    return theta1, math.pi - theta2

def rotate_point_with_matrix( r_matrix, p ):
    x_n = ( r_matrix[0][0] * p[0] ) + ( r_matrix[0][1] * p[1] ) + ( r_matrix[0][2] * p[2] )
    y_n = ( r_matrix[1][0] * p[0] ) + ( r_matrix[1][1] * p[1] ) + ( r_matrix[1][2] * p[2] )
    z_n = ( r_matrix[2][0] * p[0] ) + ( r_matrix[2][1] * p[1] ) + ( r_matrix[2][2] * p[2] )
    return np.array((x_n, y_n, z_n))

def vector_rotate_z( v, angle ):
    s = math.sin( angle )
    c = math.cos( angle )

    x = (c * v[0]) - (s * v[1])
    y = (s * v[0]) + (c * v[1])

    return( np.array((x, y, v[2])))

# Coordinates of some of the rotational joints at the actuators
rb = 195                        # vector length of all base joints from their origin
jb = 256                        # space between joints on the same side
a5 = np.array([0.0, 0.0, 0.0])
a6 = np.array([0.0, 0.0, 0.0])

# Coordinates of some of the spherical joints at the end-effector
re = 160                        #vector length of all end-effector joints from their origin
je = 45                         # space of joints in the same group
e5 = np.array([0.0, 0.0, 0.0])
e6 = np.array([0.0, 0.0, 0.0])

e_offset = -345.9365              # end-effector displacement

ANGLE = 120 * math.pi / 180

# Initial position of some easy points
base = np.zeros(18).reshape(6,3)
base[4][0] = -( rb * math.cos( math.asin( ( jb / 2.0 ) / rb) ) )
base[4][1] = jb / 2.0
base[4][2] = 0
base[5][0] = base[4][0]
base[5][1] = -base[4][1]
base[5][2] = 0

# # Rotate vector a5 ad a6 to make a1 and a2
base[0,:] = vector_rotate_z(base[4,:], ANGLE)
base[1,:] = vector_rotate_z(base[5,:], ANGLE)
# Rotate vector a5 and a6 to make a3 and a4
base[2,:] = vector_rotate_z(base[4,:], -ANGLE)
base[3,:] = vector_rotate_z(base[5,:], -ANGLE)

endeff = np.zeros(18).reshape(6,3)
endeff[4][0] = -( re * math.cos( math.asin( ( je / 2.0 ) / re ) ) )
endeff[4][1] = je / 2.0
endeff[4][2] = e_offset
endeff[5][0] = endeff[4][0]
endeff[5][1] = -endeff[4][1]
endeff[5][2] = e_offset

# # Rotate vector e5 ad e6 to make e1 and e2
endeff[0,:] = vector_rotate_z(endeff[4,:], ANGLE)
endeff[1,:] = vector_rotate_z(endeff[5,:], ANGLE)
# # Rotate vector e5 and e6 to make e3 and e4
endeff[2,:] = vector_rotate_z(endeff[4,:], -ANGLE)
endeff[3,:] = vector_rotate_z(endeff[5,:], -ANGLE)

print("Assume the origin of system fixed-frame at the center of the base and Z = 0 at the center of the joints")
print("The coordinates of all rotational joints are:")
print(" - Rotational joint 1 = ", base[0,:])
print(" - Rotational joint 2 = ", base[1,:])
print(" - Rotational joint 3 = ", base[2,:])
print(" - Rotational joint 4 = ", base[3,:])
print(" - Rotational joint 5 = ", base[4,:])
print(" - Rotational joint 6 = ", base[5,:])
print()
print("The coordinates of all spherical joints with respected to the center of the end effector are:")
print(" - Spherical joint 1 = ", endeff[0,:])
print(" - Spherical joint 2 = ", endeff[1,:])
print(" - Spherical joint 3 = ", endeff[2,:])
print(" - Spherical joint 4 = ", endeff[3,:])
print(" - Spherical joint 5 = ", endeff[4,:])
print(" - Spherical joint 6 = ", endeff[5,:])
print()
print("-----------------------------------------------------------------------------------")
print()

# We rotate frame not the vector. Therefore, the rotation angle is negative.
cos_30 = math.sqrt(3) / 2
r_matrix_j1 = np.array(((-cos_30, -0.5, 0),(-0.5, cos_30, 0),(0, 0, -1)))
r_matrix_j3 = np.array(((cos_30, -0.5, 0), (-0.5, -cos_30, 0), (0, 0, -1)))
r_matrix_j5 = np.array(((0, 1, 0), (1, 0, 0), (0, 0, -1)))

def shifted_joints( i ):
    return( endeff[i,:] - base[i,:] )

def rotated_joints( i, res ):
    if( i == 0 or i == 1 ):
        return rotate_point_with_matrix( r_matrix_j1, res )
    if( i == 2 or i == 3 ):
        return rotate_point_with_matrix( r_matrix_j3, res )
    if( i == 4 or i == 5 ):
        return rotate_point_with_matrix( r_matrix_j5, res )
    else:
        return( np.zeros(3) )

# All joint positions have the common origin point at the center of base
n_end = np.zeros(18).reshape(6,3)
ni_end = np.zeros(18).reshape(6,3)
for i in range(6):
    ni_end[i,:] = shifted_joints( i ) 
    n_end[i,:] = rotated_joints( i, ni_end[i,:] )

print("The coordinates of all end-effector joints with respected to the corresponding motor are:")
for i in range(6):
    print(" - Spherical joint", i, " after shift = ", ni_end[i,:]," after rotate = ", n_end[i,:])

print()
print("-----------------------------------------------------------------------------------")
print()

angle = np.zeros(12).reshape(6,2)
for i in range(6):
    angle[i][0], angle[i][1] = inverse_kinematic( n_end[i][0], n_end[i][1], n_end[i][2] )
    if(i % 2 == 0):
        print("Joint ", i, "Angle = ", (angle[i][0] * 180 / math.pi))
    else:
        print("Joint ", i, "Angle = ", (angle[i][1] * 180 / math.pi))

print("-----------------------------------------------------------------------------------")
print()

# plt_base = base[:,0:2]
# plt_eff = endeff[:,0:2]
# plt.plot(plt_base[:,0], plt_base[:,1])
# plt.plot(plt_eff[:,0], plt_eff[:,1])
# plt.gca().invert_yaxis()
# plt.show()
