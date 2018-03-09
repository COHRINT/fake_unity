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
        self.posePub = rospy.Publisher('pose', Pose2D, queue_size=10, latch=True)
        self.statePub = rospy.Publisher('state', RobotState, queue_size=10, latch=True)
        
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
        self.robot.posX = msg.dest.x
        self.robot.posY = msg.dest.y
        self.robot.velLin = 0.0
        self.robot.velAng = 0.0
        return []
    
    def onNewSteer(self, msg):
        print 'Showing steer:', msg.steer
        return
    
    def onJoystick(self, msg):
        print 'Axes:', msg.axes
        self.robot.velLin = msg.axes[0]
        self.robot.velAng = msg.axes[1]
        
    def run(self):
        #tick the robot state model if needed
        duration = 0.01

        '''
        self.robot.velLin = 0.1
        self.robot.theta=4*np.pi/4
        self.robot.fuelLevel = 1.0
        '''
        
        while not rospy.is_shutdown():
            self.robot.tick(duration)
            pos = Pose2D()
            #pos.header.stamp = rospy.Time.now()
            pos.x = self.robot.posX
            pos.y = self.robot.posY
            pos.theta = self.robot.theta

            self.posePub.publish(pos)

            state = RobotState()
            state.header.stamp = rospy.Time.now()
            state.pose.x = self.robot.posX
            state.pose.y = self.robot.posY
            state.pose.theta = self.robot.theta
            state.vel.linear.x = self.robot.velLin
            state.vel.angular.x = self.robot.velAng
            state.fuel = self.robot.fuelLevel
            
            self.statePub.publish(state)
            
            sleep(duration)
            #print self.robot
            
if __name__ == "__main__":
    server = FakeUnityServer()
    server.run()
    
