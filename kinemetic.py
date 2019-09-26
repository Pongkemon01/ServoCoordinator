import numpy as np 
import matplotlib.pyplot as plt

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

# Length of each arm
l1 = 95                         # Length of the actuated arms
l2 = 335                        # Length of the free arms

# Coordinates of some of the rotational joints at the actuators
rb = 220                        # vector length of all base joints from their origin
js = 256                        # space between joints on the same side
a5 = np.array([0.0, 0.0, 0.0])
a6 = np.array([0.0, 0.0, 0.0])

# Coordinates of some of the spherical joints at the end-effector
re = 160                        #vector length of all end-effector joints from their origin
je = 45                         # space of joints in the same group
e5 = np.array([0.0, 0.0, 0.0])
e6 = np.array([0.0, 0.0, 0.0])

# Coordinates of IMU with respected to the center of the end-effector
imu = np.array([160.0, 0, 0])

# Generate rotation matrix for "angle" and "-angle" to derive other joint attachment points
angle = 120.0 * (np.pi / 180.0)  # 120 degree (transform to radian)
c = np.cos(angle)
s = np.sin(angle)
R1 = np.array(((c, -s, 0), (s, c, 0), (0, 0, 1)))
c = np.cos(-angle)
s = np.sin(-angle)
R2 = np.array(((c, -s, 0), (s, c, 0), (0, 0, 1)))

# Initial position of some easy points
a5[0] = -( rb * np.cos( np.arcsin( ( js / 2.0 ) / rb) ) )
a5[1] = js / 2.0
a5[2] = 0
a6[0] = a5[0]
a6[1] = -a5[1]
a6[2] = 0

e5[0] = -( re * np.cos( np.arcsin( ( je / 2.0 ) / re ) ) )
e5[1] = je / 2.0
e5[2] = 0
e6[0] = e5[0]
e6[1] = -e5[1]
e6[2] = 0

# Generating all attachment joint positions
# Rotate vector a5 and a6 to make a3 and a4
a3 = np.dot(R2, a5)
a4 = np.dot(R2, a6)
# Rotate vector a5 ad a6 to make a1 and a2
a1 = np.dot(R1, a5)
a2 = np.dot(R1, a6)
# Rotate vector e5 and e6 to make e3 and e4
e3 = np.dot(R2, e5)
e4 = np.dot(R2, e6)
# Rotate vector e5 ad e6 to make e1 and e2
e1 = np.dot(R1, e5)
e2 = np.dot(R1, e6)

# Shift all end-effector joints to IMU origrin
ei1 = np.array(((e1[0] - imu[0]), (e1[1] - imu[1]), (e1[2] - imu[2]))) 
ei2 = np.array(((e2[0] - imu[0]), (e2[1] - imu[1]), (e2[2] - imu[2]))) 
ei3 = np.array(((e3[0] - imu[0]), (e3[1] - imu[1]), (e3[2] - imu[2]))) 
ei4 = np.array(((e4[0] - imu[0]), (e4[1] - imu[1]), (e4[2] - imu[2]))) 
ei5 = np.array(((e5[0] - imu[0]), (e5[1] - imu[1]), (e5[2] - imu[2]))) 
ei6 = np.array(((e6[0] - imu[0]), (e6[1] - imu[1]), (e6[2] - imu[2]))) 

print("Assume the origin of system fixed-frame at the center of the base and Z = 0 at the center of the joints")
print("The coordinates of all rotational joints are:")
print(" - Rotational joint 1 = ", a1)
print(" - Rotational joint 2 = ", a2)
print(" - Rotational joint 3 = ", a3)
print(" - Rotational joint 4 = ", a4)
print(" - Rotational joint 5 = ", a5)
print(" - Rotational joint 6 = ", a6)
print()
print("The coordinates of all spherical joints with respected to the center of the end effector are:")
print(" - Spherical joint 1 = ", e1)
print(" - Spherical joint 2 = ", e2)
print(" - Spherical joint 3 = ", e3)
print(" - Spherical joint 4 = ", e4)
print(" - Spherical joint 5 = ", e5)
print(" - Spherical joint 6 = ", e6)
print()
print("The actual coordinates of all spherical joints with respected to IMU are:")
print(" - Spherical joint 1 = ", ei1)
print(" - Spherical joint 2 = ", ei2)
print(" - Spherical joint 3 = ", ei3)
print(" - Spherical joint 4 = ", ei4)
print(" - Spherical joint 5 = ", ei5)
print(" - Spherical joint 6 = ", ei6)
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
q_star = q_conj( q )

# Calculate the result
re1 = np.dot(R, e1)

eq1 = np.array((0, e1[0], e1[1], e1[2]))
tre = q_multiply( eq1, q_star )
qe1 = q_multiply( q, tre )

# print result
print("The results from rotation are:")
print(" - By rotational matrix = ", re1)
print(" - By quaternion = ", qe1)
print("The rotational matrix = ", R )
print("The quaternion = ", q )
print()

#==========================
# Test simulation of movement
#==========================
yaw = 45 * (np.pi / 180.0)  # Initial yaw 45 degree
# Generate reference quaternion
qi = np.array(((np.cos(yaw/2)), 0, 0, (np.sin(yaw/2))))
qi = q_normalize( qi )
qi_star = q_conj( qi )

# Generate arbitary random quaternion
#qr = np.array(( np.random.random(),
#                np.random.random(),
#                np.random.random(),
#                np.random.random() ))
qr = np.array(((np.cos(pitch/2)), 0, (np.sin(pitch/2)), 0))
qr = q_normalize( qr )
qr_star = q_conj( qr )

# Computer relative quaternion
q_diff = np.array(((qi[0] - qr[0]), -qr[1], -qr[2], 0))
q_diff = q_normalize( q_diff )
q_diff_star = q_conj( q_diff )

# Transform
tq1 = np.array((0, e1[0], e1[1], e1[2]))
tte = q_multiply( qe1, q_diff_star )
tq1 = q_multiply ( q_diff, tte )

print("Initial quaternion is ", qi)
print("Current quaternion is ", qr)
print("Difference quaternion is ", q_diff)
print("The results from rotation is ", tq1)


#plt.Circle((0,0), radius=rb, fill=False, color="k")
#plt.plot([a1[0], a2[0], a3[0], a4[0], a5[0], a6[0], a1[0]], [a1[1], a2[1], a3[1], a4[1], a5[1], a6[1], a1[1]])
#plt.plot([a1[0], a2[0]], [a1[1], a2[1]])
#plt.plot([a3[0], a4[0]], [a3[1], a4[1]])
#plt.plot([a5[0], a6[0]], [a5[1], a6[1]])
#circle1=plt.Circle((0,0),rb,color='r', Fill=False)
#plt.gcf().gca().add_artist(circle1)
#plt.show()