#!/Users/Pongkemon/anaconda3/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import math
from scipy import signal

BISEP_LENGTH = 95.0
FORE_LENGTH = 400.0

def rotate_m( vector, matrix ):
    nx = (matrix[0][0] * vector[0]) + (matrix[0][1] * vector[1]) + (matrix[0][2] * vector[2])
    ny = (matrix[1][0] * vector[0]) + (matrix[1][1] * vector[1]) + (matrix[1][2] * vector[2])
    nz = (matrix[2][0] * vector[0]) + (matrix[2][1] * vector[1]) + (matrix[2][2] * vector[2])
    return(np.array((nx,ny,nz)))

g = np.array((0.000000,-0.100000,0.900000))
rot_matrix = np.array([[1.000000,-0.000000,0.000000], [-0.000000,0.900000,0.100000], [-0.000000,-0.100000,0.900000]])
e1 = np.array((59.719505,-148.437134,-350.000000))

print( rotate_m( e1, rot_matrix ) )
