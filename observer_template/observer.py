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

        # Create A, B, C and L matrices for lateral and Longitude

        self.prev_time = rospy.Time.now()

        self._sub_ = rospy.Subscriber('whirlybird', Whirlybird, self.whirlybirdCallback, queue_size=5)
        self.command_sub_ = rospy.Subscriber('command', Command, self.commandCallback, queue_size=5)
        #Create estimator publisher

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

        N = 10
        for i in range(0,N):
            #Euler equations for xhat lat and xhat lon

        estimator = Twist()
        # Populate estimator message with full state
        self._pub_.publish(estimator)

if __name__ == '__main__':
    rospy.init_node('observer', anonymous=True)
    # try:
    observer = Observer()
    # except:
    #     rospy.ROSInterruptException
    # pass
