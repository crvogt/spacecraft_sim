#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import numpy as np

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

class PrimeVisualizer:
    def __init__(self):
        rospy.init_node('prime_pose_visual')
        self.rate = rospy.Rate(5)

        # Values of interest
        self.angleVal = Float32()
        self.poseVal = Odometry()
        self.torqueVal = Float32()
        self.angleList = []
        self.poseList = []
        self.torqueList = []
        self.timeList = []

        # Initialize plotting object
        self.fig, self.axs = plt.subplots(3, 1, sharex=True, figsize=(10, 15))
        #self.fig, self.axs = plt.subplots(3, 1, sharex=True)

        # Subscribe to appropriate topics
        self.angleDifSub = rospy.Subscriber('prime_dif/angle_dif', Float32,self.angleDifCB)
        self.torqueSub = rospy.Subscriber('wheel_torque/wheel_torque', Float32,self.torqueCB)
        self.primePoseSub = rospy.Subscriber('prime/pose_gt', Odometry, self.primePoseCB)

    def torqueCB(self, _data):
        # Get the reaction wheel torque
        self.torqueVal = _data

    def angleDifCB(self, _data):
        # Get the solar panel angle
        self.angleVal = _data

    def primePoseCB(self, _data):
        # Get the pose of prime
        self.poseVal = _data

    def updatePlot(self):
        while not rospy.is_shutdown():
            # Scalar last
            time_val = self.poseVal.header.stamp
            self.timeList.append(time_val.to_sec())
            self.angleList.append(self.angleVal.data*180/3.14159265359)
            self.torqueList.append(self.torqueVal.data)
            
            try:
                r = R.from_quat([self.poseVal.pose.pose.orientation.x,
                                 self.poseVal.pose.pose.orientation.y,
                                 self.poseVal.pose.pose.orientation.z,
                                 self.poseVal.pose.pose.orientation.w])
                eul_angles = r.as_euler('zyx', degrees=True)
            except ValueError:
                print("Invalid quaterion, is sim paused?")
                eul_angles = [0]

            #self.poseList.append(-1*abs(eul_angles[0]))
            self.poseList.append(-1*abs(eul_angles[0]) + 180)
                

            self.axs[0].set_title('Body Angle')
            self.axs[0].set_ylabel('Degrees')
            self.axs[0].set_xlim(0.0, 50.0)
            #self.axs[0].set_ylim(-200, 10.0) 
            self.axs[0].set_ylim(0.0, 200.0) 
            #self.axs[0].plot(self.timeList, self.poseList)
            self.axs[0].plot(self.timeList, self.poseList)
            # plot time line
            self.axs[0].plot([10.02, 10.02],[-300,300], 'k--')
            self.axs[0].plot([41.6, 41.6],[-300,300], 'k--')
            #self.axs[0].scatter(time_val.to_sec(), -1*abs(eul_angles[0]), 
            #        color='black', s=2)
            
            self.axs[1].set_title('Panel Angle')
            self.axs[1].set_ylabel('Degrees')
            self.axs[1].set_ylim(-3.00, 3.00) 
            #self.axs[1].set_ylim(-90.00, 90.00) 
            self.axs[1].plot(self.timeList, self.angleList)
            # Plot time line
            self.axs[1].plot([10.02, 10.02],[-300,100], 'k--')
            self.axs[1].plot([41.6, 41.6],[-300,100], 'k--')
            #self.axs[1].scatter(time_val.to_sec(), self.angleVal.data, 
            #        color='black', s=2)

            self.axs[2].set_title('Torque Values')
            self.axs[2].set_ylabel('Newton-meters')
            self.axs[2].set_xlabel('Time (s)')
            self.axs[2].set_ylim(-0.006, 0.006) 
            self.axs[2].plot(self.timeList, self.torqueList)
            # Plot time line
            self.axs[2].plot([10.02, 10.02],[-300,100], 'k--')
            self.axs[2].plot([41.6, 41.6],[-300,100], 'k--')
            #self.axs[2].scatter(time_val.to_sec(), self.torqueVal.data,
            #        color='black', s=2)

            plt.show(block=False)
            plt.pause(0.1)
            self.axs[0].clear()
            self.axs[1].clear()
            self.axs[2].clear()

primeVis = PrimeVisualizer()
primeVis.updatePlot()
