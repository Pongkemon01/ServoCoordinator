import numpy as np
import matplotlib.pyplot as plt
import math
from scipy import signal

def normalize(v):
    l = math.sqrt((v[0]*v[0]) + (v[1]*v[1]) + (v[2]*v[2]))
    return( np.array([v[0]/l, v[1]/l, v[2]/l]))

def cross_product( u, v ):
    x = (u[1] * v[2]) - (u[2] * v[1])
    y = (u[0] * v[2]) - (u[2] * v[0])
    z = (u[0] * v[1]) - (u[1] * v[0])
    return( np.array([x, y, z]))

def genr_1(a):
    au = normalize(a)
    bu = np.array([0, 0, 1])
    R=np.array([[bu[0]*au[0], bu[0]*au[1], bu[0]*au[2]], [bu[1]*au[0], bu[1]*au[1], bu[1]*au[2]], [bu[2]*au[0], bu[2]*au[1], bu[2]*au[2]] ])
    return R

def genr_2(a):
    au = normalize(a)
    print("normalize g = ", au)
    sa = au + np.array([0, 0, 1])
    print("Biased g = ", sa)
    TR = np.array([[sa[0] * sa[0], sa[0] * sa[1], sa[0] * sa[2]], [sa[1] * sa[0], sa[1] * sa[1], sa[1] * sa[2]], [sa[2] * sa[0], sa[2] * sa[1], sa[2] * sa[2]]])
    print("raw tr  = ", TR)
    l = 1.0/((sa[0] * sa[0]) + (sa[1] * sa[1]) + (sa[2] * sa[2]))
    print("L = ", l)
    R = 2 * TR * l
    R[0][0] = R[0][0] - 1
    R[1][1] = R[1][1] - 1
    R[2][2] = R[2][2] - 1

    return R

a = np.array([0.000000, 0.099999, 0.999999])
#a = np.array([0.000000, 0, 1])
#a = np.array([0,1,0])
p = np.array([59.719997,-148.429993,-299.989990])

R1 = genr_1(a)
R2 = genr_2(a)

print(" P = ", p)

R3 = np.array(([1.0,0.0,0.0], [0.0,1.0,-0.1], [0.0,0.1,1.0]))

#print( "Matrix R1 = ", R1 )
print( "Matrix R2 = ", R2 )
print( "Matrix R3 = ", R3 )

a1 = np.matmul( R1, a )
a2 = np.matmul( R2, a )
a3 = np.matmul( R3, a )

p1 = np.matmul( R1, p )
p2 = np.matmul( R2, p )
p3 = np.matmul( R3, p )

print( "----------------")
print( "Output A1 = ", a1 )
print( "Output A2 = ", a2 )
print( "Output A3 = ", a3 )
print( "----------------")
print( "Output P1 = ", p1 )
print( "Output P2 = ", p2 )
print( "Output P3 = ", p3 )
