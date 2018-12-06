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
from whirlybird_msgs.msg import Command
from whirlybird_msgs.msg import Whirlybird
from std_msgs.msg import Float32

import numpy as np

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

        self.Fe = (m1*l1*g-m2*l2*g)/(l1)

        # Roll Gains
        tr_phi=.3#0.2
        wn_phi=2.2/tr_phi
        zeta_phi=.7#0.8
        self.phi_r = 0.0
        self.P_phi_ = wn_phi**2*Jx
        self.I_phi_ = 0.0
        self.D_phi_ = 2*zeta_phi*wn_phi*Jx
        self.Int_phi = 0.0
        self.diff_phi = 0.0
        self.prev_phi = 0.0

        # Pitch Gains
        tr_theta=1#1.2701705922171769 <- These are past values
        wn_theta=2.2/tr_theta#np.sqrt(3.0) <- These are past values
        zeta_theta=0.8#2.0/np.sqrt(3.0) <- These are past values
        self.theta_r = 0.0
        self.P_theta_ = wn_theta**2/(1.152) #Change this value to reflect kp
        self.I_theta_ = 1.2
        self.D_theta_ = 2*zeta_theta*wn_theta/(1.152) #Change this value to reflect kd
        self.prev_theta = 0.0
        self.Int_theta = 0.0
        self.diff_theta = 0.0
        self.error_theta_d1 = 0.0

        # Yaw Gains
        tr_psi=8*tr_phi
        wn_psi=2.2/tr_psi
        zeta_psi=.9#0.8
        b_psi = (l1*self.Fe)/(m1*l1**2+m2*l2**2+Jz)
        self.psi_r = 0.0
        self.P_psi_ = wn_psi**2/b_psi
        self.I_psi_ = 0.01
        self.D_psi_ = 2*zeta_psi*wn_psi/b_psi
        self.prev_psi = 0.0
        self.Int_psi = 0.0
        self.diff_psi = 0.0
        self.error_psi_d1 = 0.0

        self.prev_time = rospy.Time.now()

        #Implement your state space controller with integrator values

        #replace subscriber to whirlybird with subscriber to estimator
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
        tau_time = 0.05

        # get phi, theta, psi, phi_d, theta_d, psi_d from your msg
        # phi = msg.roll
        # theta = msg.pitch
        # psi = msg.yaw

        # Calculate dt (This is variable)
        now = rospy.Time.now()
        dt = (now-self.prev_time).to_sec()
        self.prev_time = now

        ##################################
        # Implement your controller here

        ##################################

        sat_thresh = .7

        # Scale Output
        l_out = left_force/km
        if(l_out < 0):
            l_out = 0
        elif(l_out > sat_thresh):
            l_out = sat_thresh

        r_out = right_force/km
        if(r_out < 0):
            r_out = 0
        elif(r_out > sat_thresh):
            r_out = sat_thresh

        # Pack up and send command
        command = Command()
        command.left_motor = l_out
        command.right_motor = r_out
        self.command_pub_.publish(command)


if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    controller = Controller()
    # try:
    #     controller = Controller()
    # except:
    #     rospy.ROSInterruptException
    pass
