import numpy as np 
import matplotlib.pyplot as plt
import math

def q_multiply( q1, q2 ):
    a = (q1[0]*q2[0]) - (q1[1]*q2[1]) - (q1[2]*q2[2]) - (q1[3]*q2[3])
    b = (q1[0]*q2[1]) + (q1[1]*q2[0]) + (q1[2]*q2[3]) - (q1[3]*q2[2])
    c = (q1[0]*q2[2]) - (q1[1]*q2[3]) + (q1[2]*q2[0]) + (q1[3]*q2[1])
    d = (q1[0]*q2[3]) + (q1[1]*q2[2]) - (q1[2]*q2[1]) + (q1[3]*q2[0])
    return( np.array((a, b, c, d)))

def q_normalize( q ):
    q_norm = np.sqrt( (q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2]) + (q[3] * q[3]) )
    return( q / q_norm )

def q_conj( q ):
    return( np.array((q[0], -q[1], -q[2], -q[3])) )

def gen_rotation_matrix ( a ):   # a is np.array of yaw,pitch,roll respectively
    s1 = np.sin(a[0])
    c1 = np.cos(a[0])
    s2 = np.sin(a[1])
    c2 = np.cos(a[1])
    s3 = np.sin(a[2])
    c3 = np.cos(a[2])
    return( np.array (( (c1*s2, (c1*s2*s3 - c3*s1), (s1*s3 + c1*c3*s2)), (c2*s1, (c1*c3 + s1*s2*s3), (c3*s1*s2 - c1*s3)), (-s2, c2*s3, c2*c3) )))

def q_rotate( vector, quaternion ):
    return( q_multiply( q_multiply(quaternion, vector), q_conj(quaternion) ) )

def rotate_with_matrix( r_matrix, x, y, z ):
    x_n = ( r_matrix[0][0] * x ) + ( r_matrix[0][1] * y ) + ( r_matrix[0][2] * z )
    y_n = ( r_matrix[1][0] * x ) + ( r_matrix[1][1] * y ) + ( r_matrix[1][2] * z )
    z_n = ( r_matrix[2][0] * x ) + ( r_matrix[2][1] * y ) + ( r_matrix[2][2] * z )
    return x_n, y_n, z_n

# Length of each arm
BISEP_LENGTH = 94                         # Length of the actuated arms
FORE_LENGTH = 400                        # Length of the free arms

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

e_offset = -357.54              # end-effector displacement

# Generate rotation matrix for "angle" and "-angle" to derive other joint attachment points
angle = 120.0 * (np.pi / 180.0)  # 120 degree (transform to radian)
c = np.cos(angle)
s = np.sin(angle)
R1 = np.array(((c, -s, 0), (s, c, 0), (0, 0, 1)))
c = np.cos(-angle)
s = np.sin(-angle)
R2 = np.array(((c, -s, 0), (s, c, 0), (0, 0, 1)))

# Initial position of some easy points
base = np.zeros(18).reshape(6,3)
base[4][0] = -( rb * np.cos( np.arcsin( ( jb / 2.0 ) / rb) ) )
base[4][1] = jb / 2.0
base[4][2] = 0
base[5][0] = base[4][0]
base[5][1] = -base[4][1]
base[5][2] = 0

endeff = np.zeros(18).reshape(6,3)
endeff[4][0] = -( re * np.cos( np.arcsin( ( je / 2.0 ) / re ) ) )
endeff[4][1] = je / 2.0
endeff[4][2] = e_offset
endeff[5][0] = endeff[4][0]
endeff[5][1] = -endeff[4][1]
endeff[5][2] = e_offset

# Generating all attachment joint positions
# Rotate vector a5 and a6 to make a3 and a4
base[2,:] = np.dot(R2, base[4,:])
base[3,:] = np.dot(R2, base[5,:])
# Rotate vector a5 ad a6 to make a1 and a2
base[0,:] = np.dot(R1, base[4,:])
base[1,:] = np.dot(R1, base[5,:])
# Rotate vector e5 and e6 to make e3 and e4
endeff[2,:] = np.dot(R2, endeff[4,:])
endeff[3,:] = np.dot(R2, endeff[5,:])
# Rotate vector e5 ad e6 to make e1 and e2
endeff[0,:] = np.dot(R1, endeff[4,:])
endeff[1,:] = np.dot(R1, endeff[5,:])

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

#==========================
# Test basic transformation
#==========================
pitch = 30 * (np.pi / 180.0)  # pitch 30 degree

# Generate rotation matrix
c = np.cos(pitch)
s = np.sin(pitch)
R = np.array(((c, 0, s), (0, 1, 0), (-s, 0, c)))

# Generate quaternion
q = np.array(((np.cos(pitch/2)), 0, (np.sin(pitch/2)), 0))
q = q_normalize( q )

# Calculate the result
re1 = np.dot(R, endeff[0,:])
qe1 = q_rotate( np.array((0, endeff[0][0], endeff[0][1], endeff[0][2])), q )

# print result
print("The original vector is ", endeff[0,:])
print("The results from rotation are:")
print(" - By rotational matrix = ", re1)
print(" - By quaternion = ", qe1)
print("The rotational matrix = ", R )
print("The quaternion = ", q )
print("-----------------------------------------------------------------------------------")
print()

#==========================
# Test anti-quaternion
#==========================
# Initial vector
iv = np.array((0, endeff[0][0], endeff[0][1], endeff[0][2]))
# Generate reference quaternion
qr = np.array(((np.cos(pitch/2)), 0, (np.sin(pitch/2)), 0))
qr = q_normalize( qr )

# Generate first reference point
rv = q_rotate( iv, qr )

yaw = 45 * (np.pi / 180.0)  # Initial yaw 45 degree
# Generate control rotation quaternion
qi = np.array(((np.cos(yaw/2)), 0, 0, (np.sin(yaw/2))))
qi = q_normalize( qi )

# Generate current vector
cv = q_rotate( iv, qi )

# Generate anti-quaternion to make cv becoming rv
# aq = qr / qi  ==>   aq = ar * qi'
aq = q_multiply( qr, q_conj(qi) )
aq = q_normalize(aq)

# Rotate cv with aq to get the final vector
aqv = q_rotate( cv, aq )

print("Initial quaternion is ", qi)
print("Current quaternion is ", qr)
print("Anti-quaternion is ", aq)
print("The reference vector is ", rv)
print("Current vector is ", cv)
print("The results from anti-quaternion is ", aqv)
print("-----------------------------------------------------------------------------------")
print()

#plt.Circle((0,0), radius=rb, fill=False, color="k")
#plt.plot([a1[0], a2[0], a3[0], a4[0], a5[0], a6[0], a1[0]], [a1[1], a2[1], a3[1], a4[1], a5[1], a6[1], a1[1]])
#plt.plot([a1[0], a2[0]], [a1[1], a2[1]])
#plt.plot([a3[0], a4[0]], [a3[1], a4[1]])
#plt.plot([a5[0], a6[0]], [a5[1], a6[1]])
#circle1=plt.Circle((0,0),rb,color='r', Fill=False)
#plt.gcf().gca().add_artist(circle1)
#plt.show()

#==========================
# Test inverse kinematic
#==========================

cos_30 = math.sqrt(3) / 2
r_matrix_j1 = np.array(((-cos_30, 0.5, 0),(0.5, cos_30, 0),(0, 0, -1)))
r_matrix_j3 = np.array(((0, -1, 0), (-1, 0, 0), (0, 0, -1)))
r_matrix_j5 = np.array(((cos_30, 0.5, 0), (0.5, -cos_30, 0), (0, 0, -1)))

def inverse_kinematic( x, y, z ):
    f_dat = FORE_LENGTH * math.cos( math.asin( y / FORE_LENGTH ) )
    theta_phi = math.atan2( z, x )
    print(theta_phi)
    cos_phi = (x * x) + (z * z) + (BISEP_LENGTH * BISEP_LENGTH) - (f_dat * f_dat)
    cos_phi /= ( 2.0 * BISEP_LENGTH * math.sqrt( (x * x) + (z * z) ) )
    phi = math.acos( cos_phi )
    return (theta_phi - phi), (theta_phi + phi)

def rotate_point_with_matrix( r_matrix, p ):
    x_n = ( r_matrix[0][0] * p[0] ) + ( r_matrix[0][1] * p[1] ) + ( r_matrix[0][2] * p[2] )
    y_n = ( r_matrix[1][0] * p[0] ) + ( r_matrix[1][1] * p[1] ) + ( r_matrix[1][2] * p[2] )
    z_n = ( r_matrix[2][0] * p[0] ) + ( r_matrix[2][1] * p[1] ) + ( r_matrix[2][2] * p[2] )
    return np.array((x_n, y_n, z_n))

def shifted_joints( i ):
    res = endeff[i,:] - base[i,:]
    if( i == 0 or i == 1 ):
        return rotate_point_with_matrix( r_matrix_j1, res )
    if( i == 2 or i == 3 ):
        return rotate_point_with_matrix( r_matrix_j3, res )
    else:
        return rotate_point_with_matrix( r_matrix_j1, res )

t0 = shifted_joints( 0 )
t1 = shifted_joints( 1 )

t0_t1, t0_t2 = inverse_kinematic( t0[0], t0[1], t0[2] )
t1_t1, t1_t2 = inverse_kinematic( t1[0], t1[1], t1[2] )

print("Translate joint 0 ", endeff[0,:])
print("Base joint 0 ", base[0,:])
print("Point becomes ", t0)
print("Angle E1/1 = ", (t0_t1 * 180 / math.pi))
print("Angle E1/2 = ", (t0_t2 * 180 / math.pi))
print("========================")
print("Translate joint 1 ", endeff[1,:])
print("Base joint 1 ", base[1,:])
print("Point becomes ", t1)
print("Angle E2/1 = ", (t1_t1 * 180 / math.pi))
print("Angle E2/2 = ", (t1_t2 * 180 / math.pi))
print("-----------------------------------------------------------------------------------")
print()
