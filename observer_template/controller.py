#!/usr/bin/env python
# license removed for brevity


# This file is a basic structure to write a controller that
# communicates with ROS. It will be the students responsibility
# tune the gains and fill in the missing information

# As an example this file contains PID gains, but other
# controllers use different types of gains so the class
# will need to be modified to accomodate those changes

import rospy
import time
from math import cos,pi,sqrt
import numpy as np
from whirlybird_msgs.msg import Command
from whirlybird_msgs.msg import Whirlybird
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from scipy import signal
import scipy.signal as ss
from math import sin,cos



class Controller():

    def __init__(self):

        # get parameters
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


        # Roll Gains
	#################Lateral####################################
	############################################################
	theta = 0.0
	self.Fe = (m1*l1-m2*l2)*g*cos(0)/l1
	Alat = np.matrix([[0,0,1,0],
			[0,0,0,1],
			[0,0,0,0],
			[l1*self.Fe/(m1*l1**2+m2*l2**2+Jz),0,0,0]])
	Blat = np.matrix([[0],
			[0],
			[1/Jx],
			[0]])
	Clat = np.matrix([[1,0,0,0],
			[0,1,0,0]])

	Cr = Clat[1,:]

	## augment the matrices
	Alat = np.hstack((np.vstack((Alat,-Cr)),np.zeros((5,1))))
	Blat = np.vstack((Blat,0))


	# phi (roll)
	tr_phi = 0.3   ## roll rise time

	zeta_phi = 0.7
	wn_phi = pi/2.0/tr_phi/sqrt(1-zeta_phi**2)

	# psi (yaw)
	tr_psi = tr_phi*9.0    ## yaw rise time 6X longer than roll rise time

	zeta_psi = 0.7
	wn_psi = pi/2.0/tr_psi/sqrt(1-zeta_psi**2)

	kI_pole = -wn_phi/2.0
	 
	plat = np.array([-zeta_phi*wn_phi + 1j*wn_phi*sqrt(1-zeta_phi**2), \
			 -zeta_phi*wn_phi - 1j*wn_phi*sqrt(1-zeta_phi**2), \
			 -zeta_psi*wn_psi + 1j*wn_psi*sqrt(1-zeta_psi**2), \
	 		 -zeta_psi*wn_psi - 1j*wn_psi*sqrt(1-zeta_psi**2), \
				kI_pole])

	K = ss.place_poles(Alat,Blat,plat).gain_matrix
	self.kIlat = K.item(4)
	self.Klat = np.matrix([K.item(0),K.item(1),K.item(2),K.item(3)])



	#################Longitudinal#############################
	############################################################
	Alon = np.matrix([[0,1],
		         [(m1*l1-m2*l2)*g*sin(theta)/(m1*l1**2+m2*l2**2+Jy),0]])

	Blon = np.matrix([[0],
		         [l1/(m1*l1**2+m2*l2**2+Jy)]])

	Clon = np.matrix([[1,0]])


	## augment the matrices

	Alon = np.hstack((np.vstack((Alon,-Clon)),np.zeros((3,1))))
	Blon = np.vstack((Blon,0))


	## place poles for longitudinal
	####################################
	# theta (pitch)
	tr_theta = 1.0
	zeta_theta = 0.7
	wn_theta = pi/2.0/tr_theta/sqrt(1-zeta_theta**2)

	kI_pole = -wn_theta/2.0

	## desired poles
	#########################################
	plon = np.array([-zeta_theta*wn_theta + 1j*wn_theta*sqrt(1-zeta_theta**2),\
		         -zeta_theta*wn_theta - 1j*wn_theta*sqrt(1-zeta_theta**2), \
					kI_pole] )


	## place poles for longitudinal
	####################################
	K = ss.place_poles(Alon,Blon,plon).gain_matrix

	self.kIlon = K.item(2)
	self.Klon = np.matrix([K.item(0),K.item(1)])


        self.P_phi_ = 0.0
        self.I_phi_ = 0.0
        self.D_phi_ = 0.0
        self.Int_phi = 0.0
        self.prev_phi = 0.0
	self.phid = 0.0 
	self.phi_integrator = 0.0
	self.phi_error_prev = 0.0

        # Pitch Gains
        self.theta_r = 0.0
        self.P_theta_ = 0.0
        self.I_theta_ = 0.0
        self.D_theta_ = 0.0
        self.prev_theta = 0.0
        self.Int_theta = 0.0
	self.thetad = 0.0 
	self.theta_integrator = 0.0
	self.theta_error_prev = 0.0


        # Yaw Gains
        self.psi_r = 0.0
        self.P_psi_ = 0.0
        self.I_psi_ = 0.0
        self.D_psi_ = 0.0
        self.prev_psi = 0.0
        self.Int_psi = 0.0
	self.psid = 0.0 
	self.psi_integrator = 0.0
	self.psi_error_prev = 0.0


        self.prev_time = rospy.Time.now()

	self.Ftil = 0.0
	self.tau = 0.0

        self.psi_r_sub_ = rospy.Subscriber('psi_r', Float32, self.psiRCallback, queue_size=5)
        self.theta_r_sub_ = rospy.Subscriber('theta_r', Float32, self.thetaRCallback, queue_size=5)
	self.observer_sub_ = rospy.Subscriber('estimator', Twist, self.observerCallback, queue_size=5)
        self.command_pub_ = rospy.Publisher('command', Command, queue_size=5)
        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()


    def thetaRCallback(self, msg):
        self.theta_r = msg.data


    def psiRCallback(self, msg):
        self.psi_r = msg.data


    def observerCallback(self, msg):

	# unpack the msg
	theta = msg.linear.x
	phi = msg.linear.y
        psi = msg.linear.z

	self.thetad = msg.angular.x
	self.phid = msg.angular.y
	self.psid = msg.angular.z

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



        # Calculate dt (This is variable)
        now = rospy.Time.now()
        dt = (now-self.prev_time).to_sec()
        self.prev_time = now
        
        ##################################
        # Implement your controller here

	####### Integrator theta ###########
	theta_error = self.theta_r - theta
	#Anti windup
	if abs(self.thetad) < 0.2:
		# integrate error for theta
		self.theta_integrator += dt*(theta_error + self.theta_error_prev)/2.0

	self.theta_error_prev = theta_error


	##### state vector longitude #########
	xlon = np.matrix([[theta],[self.thetad]])



	###### calculate F #########
	F = - self.Klon*xlon - self.kIlon*self.theta_integrator
	
	
	####### equillibrium force ########
	self.Fe = (m1*l1-m2*l2)*g*cos(theta)/l1   # we added the 0.85
	F += self.Fe



	######## Integrator psi ################
	psi_error = self.psi_r - psi
	#Anti windup
	if abs(self.psid) < 0.05:
		# integrate error for psi
		self.psi_integrator += dt*(psi_error + self.psi_error_prev)/2.0

	self.psi_error_prev = psi_error # update previous error



	###### state vector for lateral
	xlat = np.matrix([[phi],[psi],[self.phid],[self.psid]])
	self.tau = - self.Klat*xlat - self.kIlat*self.psi_integrator
	
	

        ##################################

        # Scale Output
        l_out = (F+self.tau/d)/2.0/km  ## convert to PWM
        if(l_out < 0):
            l_out = 0
        elif(l_out > 0.7):
            l_out = 0.7

        r_out = (F-self.tau/d)/2.0/km
        if(r_out < 0):
            r_out = 0
        elif(r_out > 0.7):
            r_out = 0.7
	

	# Pack up and send command
        command = Command()
        command.left_motor = l_out
        command.right_motor = r_out
        self.command_pub_.publish(command)


if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
#    try:
    controller = Controller()
#    except:
 #       rospy.ROSInterruptException
#    pass
