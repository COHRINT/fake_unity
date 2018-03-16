#!/usr/bin/python2

'''
ROS node to emulate Unity for test and debug purposes

Maintains a really dumb vehicle dynamics model for movement from point to point
Subscribes:
Steering (Directions from the MDP)

Publishes:
Pose2D (current position)
FuelLevel (current fuel)


Services: 
ShowToast: Show user interface 
SetFuelLevel: Reset the fuel dynamics to something
Teleport: Move the robot

'''

import rospy
from traadre_msgs.msg import *
from traadre_msgs.srv import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
import tf

import sys, pickle
import numpy as np
import cv_bridge
import cv2
import math
from time import sleep

class RobotSim(object):
    def __init__(self):
        self.velLin = 0.0
        self.velAng = 0.0
        
        self.posX = 0.0
        self.posY = 0.0
        self.theta = 0.0

        self.fuelLevel = 0.0
        
    def wrapToPi(self, val):
        return np.mod(val+np.pi,2*np.pi)-np.pi
    
    def tick(self, duration):
        #Tick the model one timestep:
        dist = duration*self.velLin
        
        if self.fuelLevel < 0.0:
            self.velLin = 0.0
            self.velAng = 0.0
            self.fuelLevel = 0.0
            
        else:
            self.fuelLevel -= np.fabs(duration*dist*100) #linear fuel model


        self.theta = self.wrapToPi(self.theta + self.velAng*duration)
        self.posX += math.cos(self.theta)*dist
        self.posY += math.sin(self.theta)*dist

    def __str__(self):
        return 'X: %1.3f Y: %1.3f Theta: %1.3f Fuel:%1.3f' % (self.posX, self.posY, self.theta, self.fuelLevel)
        
class FakeUnityServer(object):
    def __init__(self):
        print 'Starting fake Unity server'
        self.nodeName = 'fake_unity_server'
        rospy.init_node(self.nodeName)

        self.fuelSrv = rospy.Service('~SetFuelLevel', SetFuelLevel, self.setFuelLevel)
        self.toastSrv = rospy.Service('~ShowToast', ShowToast, self.showToast)
        self.teleportSrv = rospy.Service('~Teleport', Teleport, self.teleport)

        
        self.steerSub = rospy.Subscriber('steer', Steering, self.onNewSteer)
        self.joySub = rospy.Subscriber('joy', Joy, self.onJoystick)
        self.posePub = rospy.Publisher('pose', PoseStamped, queue_size=10, latch=True)
        self.statePub = rospy.Publisher('state', RobotState, queue_size=10, latch=True)

        self.stateSub = rospy.Subscriber('state_cmd', RobotState, self.onStateCmd)
        
        #Dynamics model
        self.robot = RobotSim()
        
        print 'Fake Unity server ready!'
        
    def setFuelLevel(self, msg):
        self.robot.fuelLevel = msg.fuelLevel
        return []
    
    def showToast(self, msg):
        print 'Message for %d secs:' % msg.duration, msg.text
        return []
    
    def teleport(self, msg):
        self.robot.posX = msg.dest.pose.position.x
        self.robot.posY = msg.dest.pose.position.y
        self.robot.velLin = 0.0
        self.robot.velAng = 0.0
        return []
    
    def onNewSteer(self, msg):
        print 'Showing steer:', msg.steer
        return
    
    def onJoystick(self, msg):
        #print 'Axes:', msg.axes
        self.robot.velLin = 10.0*msg.axes[1]
        self.robot.velAng = -msg.axes[0]

    def onStateCmd(self, msg):
        #Use a RobotState message to move the internal model as needed
        print 'Updating internal state per command'

        self.robot.posX = msg.pose.position.x 
        self.robot.posY = msg.pose.position.y
        worldRoll, worldPitch, worldYaw = euler_from_quaternion([msg.pose.orientation.w,
                                                                 msg.pose.orientation.x,
                                                                 msg.pose.orientation.y,
                                                                 msg.pose.orientation.z],'sxyz')
        self.robot.theta = worldYaw
        self.robot.fuelLevel = msg.fuelLevel

        
    def run(self):
        #tick the robot state model if needed
        duration = 0.01

        '''
        self.robot.velLin = 0.1
        self.robot.theta=4*np.pi/4
        
        '''
        self.robot.fuelLevel = 100.0
        
        while not rospy.is_shutdown():
            self.robot.tick(duration)

            pos = PoseStamped()
            pos.header.stamp = rospy.Time.now()
            pos.pose.position.x = self.robot.posX
            pos.pose.position.y = self.robot.posY

            quat = tf.transformations.quaternion_from_euler(0.0, 0.0, self.robot.theta, 'rxyz') #from rpy
            pos.pose.orientation.x = quat[1]
            pos.pose.orientation.y = quat[2]
            pos.pose.orientation.z = quat[3]
            pos.pose.orientation.w = quat[0]
            
            self.posePub.publish(pos)

            state = RobotState()
            state.header.stamp = rospy.Time.now()
            state.pose.position.x = self.robot.posX
            state.pose.position.y = self.robot.posY

            state.pose.orientation.x = quat[1]
            state.pose.orientation.y = quat[2]
            state.pose.orientation.z = quat[3]
            state.pose.orientation.w = quat[0]
            state.vel.linear.x = self.robot.velLin
            state.vel.angular.x = self.robot.velAng
            state.fuel = self.robot.fuelLevel
            
            self.statePub.publish(state)
            
            sleep(duration)
            #print self.robot
            
if __name__ == "__main__":
    server = FakeUnityServer()
    server.run()
    
