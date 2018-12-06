#!/usr/bin/env python
# license removed for brevity


import rospy
import time
import numpy as np
from whirlybird_msgs.msg import Command
from whirlybird_msgs.msg import Whirlybird
from geometry_msgs.msg import Twist
# from whirlybird_msgs.msg import
from std_msgs.msg import Float32
import control
from scipy import signal
import scipy.signal as ss
from math import sin,cos

class Observer():

    def __init__(self):
        try:
            param_namespace = '/whirlybird'
            self.param = rospy.get_param(param_namespace)
        except KeyError:
            rospy.logfatal('Parameters not set in ~/whirlybird namespace')
            rospy.signal_shutdown('Parameters not set')

        g = self.param['g']
        l1 = self.param['l1']
        l2 = self.param['l2']
        m1 = self.param['m1']
        m2 = self.param['m2']
        d = self.param['d']
        h = self.param['h']
        r = self.param['r']
        Jx = self.param['Jx']
        Jy = self.param['Jy']
        Jz = self.param['Jz']
        km = self.param['km']

        self.r_in = 0.0
        self.l_in = 0.0

        self.theta_e = 0
        self.Fe = (m1*l1 - m2*l2)*(g/l1)

        # initialize xhat for lat and lon
        # specify rise time, damping, omega, and observer omega for theta, phi and psi
	tr_phi = .02   # 5 times faster than controller rise times
	tr_psi = .04	
	tr_theta = .02

	zeta = 0.707
	wn_phi = 2.2/tr_phi
	wn_psi = 2.2/tr_psi
	wn_theta = 2.2/tr_theta
	theta = 0.0
        # Create A, B, C and L matrices for lateral and Longitude
	self.Alon = np.matrix([[0,1],
                 [(m1*l1-m2*l2)*g*sin(theta)/(m1*l1**2+m2*l2**2+Jy),0]])

	self.Blon = np.matrix([[0],
                 [l1/(m1*l1**2+m2*l2**2+Jy)]])

	self.Clon = np.matrix([[1,0]])

	desiredpoles_lon = np.roots([1,2*wn_theta*zeta,zeta**2])
	self.Llon = ss.place_poles(self.Alon.T,self.Clon.T,desiredpoles_lon).gain_matrix
	
	self.Llon = self.Llon.T
	print(self.Llon)

	
	Fe = 0.6*(m1*l1-m2*l2)*g*cos(theta)/l1

	self.Alat = np.matrix([[0,0,1,0],
			[0,0,0,1],
			[0,0,0,0],
			[l1*Fe/(m1*l1**2+m2*l2**2+Jz),0,0,0]])

	self.Blat = np.matrix([[0],
			[0],
			[1/Jx],
			[0]])

	self.Clat = np.matrix([[1,0,0,0],
			[0,1,0,0]])


	desiredpoles_lat = np.roots(np.convolve([1,2*wn_psi*zeta,zeta**2],[1,2*wn_phi*zeta,zeta**2]))
	self.Llat = ss.place_poles(self.Alat.T,self.Clat.T,desiredpoles_lat).gain_matrix
	self.Llat = self.Llat.T

	self.xhat_lon = np.matrix([[0.],[0.]])
	self.xhat_lat = np.matrix([[0.],[0.],[0.],[0.]])


        self.prev_time = rospy.Time.now()

        self._sub_ = rospy.Subscriber('whirlybird', Whirlybird, self.whirlybirdCallback, queue_size=5)
        self.command_sub_ = rospy.Subscriber('command', Command, self.commandCallback, queue_size=5)
        #self.psi_r_sub_ = rospy.Subscriber('psi_r', Float32, self.psiRCallback, queue_size=5)
        #self.theta_r_sub_ = rospy.Subscriber('theta_r', Float32, self.thetaRCallback, queue_size=5)
        #Create estimator publisher
	self._pub_ = rospy.Publisher('estimator', Twist, queue_size=5)

        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()

    def commandCallback(self, msg):
        self.r_in = msg.right_motor
        self.l_in = msg.left_motor

    def whirlybirdCallback(self, msg):

        g = self.param['g']
        l1 = self.param['l1']
        l2 = self.param['l2']
        m1 = self.param['m1']
        m2 = self.param['m2']
        d = self.param['d']
        h = self.param['h']
        r = self.param['r']
        Jx = self.param['Jx']
        Jy = self.param['Jy']
        Jz = self.param['Jz']
        km = self.param['km']

        now = rospy.Time.now()
        dt = (now-self.prev_time).to_sec()
        self.prev_time = now

        phi_m = msg.roll
        theta_m = msg.pitch
        psi_m = msg.yaw
        # Calculate F, tau, and equilibrium force

	

	force_prev = km*(self.l_in+self.r_in)
	tau_prev = km*d*(self.l_in-self.r_in)
	Fe = 0.85*(m1*l1-m2*l2)*g*cos(theta_m)/l1   # we added the 0.85

	ylon = np.matrix([[theta_m]])         # measured states
	ylat = np.matrix([[phi_m],[psi_m]])   # measured states
	
	
	
	
        N = 10
        for i in range(0,N):
		#Euler equations for xhat lat and xhat lon
		self.xhat_lon += dt/float(N)*(self.Alon*self.xhat_lon+self.Blon*(force_prev-Fe) + self.Llon*(ylon-self.Clon*self.xhat_lon))
		self.xhat_lat += dt/float(N)*(self.Alat*self.xhat_lat+self.Blat*tau_prev + self.Llat*(ylat-self.Clat*self.xhat_lat))
		pass
            
        estimator = Twist()
        # Populate estimator message with full state
	estimator.linear.x = self.xhat_lon[0][0]

	estimator.linear.y = self.xhat_lat[0][0]
	estimator.linear.z = self.xhat_lat[1][0]

	estimator.angular.x = self.xhat_lon[1][0]

	estimator.angular.y = self.xhat_lat[2][0]
	estimator.angular.z = self.xhat_lat[3][0]
        self._pub_.publish(estimator)

if __name__ == '__main__':
    rospy.init_node('observer', anonymous=True)
    # try:
    observer = Observer()
    # except:
    #     rospy.ROSInterruptException
    # pass
