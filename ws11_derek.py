# WS 11 Lab assignment

import numpy as np
import matplotlib.pyplot as plt
from control.matlab import *

'''
Alat = np.matrix([0,0,1,0],
                 [0,0,0,1],
                 [0,0,0,0],
                 [l1*Fe/1,0,0,0])
'''
Alon = np.matrix([0,1],
                 [(m1*l1-m2*l2)*g*cos(theta)/(m1*l1**2+m2*l2**2+Jy),0])
Blon = np.matrix([0],
                 [l1/(m1*l1**2+m2*l2**2+Jy)])
Clon = np.matrix([1,0])

plon = np.array([-zeta_theta*wn_theta+i*wn_theta*sqrt(1-zeta**2),\
-zeta_theta*wn_theta-i*wn_theta*sqrt(1-zeta**2)])
