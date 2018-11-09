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
from std_msgs.msg import Float32


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

        self.Fe = 0.0 #Note this is not the correct value for Fe, you will have to find that yourself
	self.Ftil = 0.0
	self.tau = 0.0

        self.command_sub_ = rospy.Subscriber('whirlybird', Whirlybird, self.whirlybirdCallback, queue_size=5)
        self.psi_r_sub_ = rospy.Subscriber('psi_r', Float32, self.psiRCallback, queue_size=5)
        self.theta_r_sub_ = rospy.Subscriber('theta_r', Float32, self.thetaRCallback, queue_size=5)
        self.command_pub_ = rospy.Publisher('command', Command, queue_size=5)
        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()


    def thetaRCallback(self, msg):
        self.theta_r = msg.data


    def psiRCallback(self, msg):
        self.psi_r = msg.data


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

        phi = msg.roll
        theta = msg.pitch
        psi = msg.yaw

        # Calculate dt (This is variable)
        now = rospy.Time.now()
        dt = (now-self.prev_time).to_sec()
        self.prev_time = now
        
        ##################################
        # Implement your controller here
	
	# Dirty derivatve for theta
	sigma = 0.05  # cutoff freq for dirty derivative
	beta = (2.0*sigma-dt)/(2.0*sigma+dt)  # dirty derivative gain
	self.thetad = beta*self.thetad + (1-beta)*(theta - self.prev_theta)/dt
	self.prev_theta = theta # update previous theta

	# state vector
	x = np.matrix([[theta],[self.thetad]])
	Klon = np.matrix([[7.36624816, 1.9098619 ]]) 
	krlon = 2.14328874  

	F =  krlon*self.theta_r - Klon*x
	print F,
	self.Fe = 1.0*(m1*l1-m2*l2)*g*cos(theta)/l1
	F += self.Fe

	
	print self.Fe,
	# Dirty derivatve for psi
	self.psid = beta*self.psid + (1-beta)*(psi - self.prev_psi)/dt
	self.prev_psi = psi # update previous psi

	# Dirty derivatve for phi
	self.phid = beta*self.phid + (1-beta)*(phi - self.prev_phi)/dt
	self.prev_phi = phi # update previous phi


	Klat = np.matrix([[0.34220482, 0.10491429, 0.05628416, 0.14023202]])
	krlat = 0.10491429

	xlat = np.matrix([[phi],[psi],[self.phid],[self.psid]])
	self.tau = krlat*self.psi_r - Klat*xlat
	
	print self.tau
	
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
    try:
        controller = Controller()
    except:
        rospy.ROSInterruptException
    pass
