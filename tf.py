import numpy as np
from sympy import *

s,z,T,w,c,l,p = symbols( 's z T w c l p' )

c = 0.37 * w
l = w / 0.37
p = 0.5 * sqrt(2)
s = (2/T)*((z-1)/(z+1))

den_h = (s**2)+(np.sqrt(2)*c*s)+(c**2)
den_l = (s**2)+(np.sqrt(2)*l*s)+(l**2)
nom_h = c**2
nom_l = (l**2)*(s**2)

HL = nom_l / den_l
HH = nom_h / den_h

hl1 = expand(HL)
hl2 = factor(hl1)
hl3 = simplify(hl2)
hl4 = collect(hl3, z)

hh1 = expand(HH)
hh2 = factor(hh1)
hh3 = simplify(hh2)
hh4 = collect(hh3, z)
