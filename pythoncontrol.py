import numpy as np
import matplotlib.pyplot as plt
from control.matlab import *
from math import cos,sin,sqrt,pi


l1= 0.85
l2= 0.3048
m1= 0.891
m2= 1.0
d= 0.178
h= 0.65
r= 0.12
Jx= 0.0047
Jy= 0.0014
Jz= 0.0041
g = 9.81


#################Lateral####################################
############################################################
theta = 0.0
Fe = 0.6*(m1*l1-m2*l2)*g*cos(theta)/l1

Alat = np.matrix([[0,0,1,0],
		[0,0,0,1],
		[0,0,0,0],
		[l1*Fe/(m1*l1**2+m2*l2**2+Jz),0,0,0]])

Blat = np.matrix([[0],
		[0],
		[1/Jx],
		[0]])

Clat = np.matrix([[1,0,0,0],
		[0,1,0,0]])

Dlat = np.matrix([[0],[0]])




## determine controllability
####################################
C_ABlat = ctrb(Alat,Blat)
Ranklat = np.linalg.matrix_rank(C_ABlat)




## place poles for lateral
####################################
# phi (roll)
tr_phi = 0.3   ## roll rise time

zeta_phi = 0.7
wn_phi = pi/2.0/tr_phi/sqrt(1-zeta_phi**2)


# psi (yaw)
tr_psi = tr_phi*6.0    ## yaw rise time 6X longer than roll rise time

zeta_psi = 0.7
wn_psi = pi/2.0/tr_psi/sqrt(1-zeta_psi**2)


plat = np.array([-zeta_phi*wn_phi + 1j*wn_phi*sqrt(1-zeta_phi**2), \
		 -zeta_phi*wn_phi - 1j*wn_phi*sqrt(1-zeta_phi**2), \
		 -zeta_psi*wn_psi + 1j*wn_psi*sqrt(1-zeta_psi**2), \
 		 -zeta_psi*wn_psi - 1j*wn_psi*sqrt(1-zeta_psi**2)])

Klat = place(Alat,Blat,plat)
Cyaw = np.matrix([[0,1,0,0]])
krlat = -1/(Cyaw*np.linalg.inv(Alat-Blat*Klat)*Blat)


sys = ss(Alat,Blat,Clat,Dlat)











#################Longitudinal#############################
############################################################
Alon = np.matrix([[0,1],
                 [(m1*l1-m2*l2)*g*sin(theta)/(m1*l1**2+m2*l2**2+Jy),0]])

Blon = np.matrix([[0],
                 [l1/(m1*l1**2+m2*l2**2+Jy)]])

Clon = np.matrix([[1,0]])


## determine controllability
C_ABlon = ctrb(Alon,Blon)
Ranklon = np.linalg.matrix_rank(C_ABlon)


## place poles for longitudinal
####################################
# theta (pitch)
tr_theta = 1.4
zeta_theta = 0.7
wn_theta = pi/2.0/tr_theta/sqrt(1-zeta_theta**2)


## desired poles
#########################################
plon = np.array([-zeta_theta*wn_theta + 1j*wn_theta*sqrt(1-zeta_theta**2),\
                 -zeta_theta*wn_theta - 1j*wn_theta*sqrt(1-zeta_theta**2)])


## place poles for longitudinal
####################################
Klon = place(Alon,Blon,plon)
krlon = -1/(Clon*np.linalg.inv(Alon-Blon*Klon)*Blon)


for key,value in globals().items():
    if not key.startswith('_') and not callable(value) and not key == 'np':
        print key,value,"\n"*5
	





