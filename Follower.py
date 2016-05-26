#!/usr/bin/env python


# Import all of the basic ROS functions, and make sure that the Python path is set
# up properly
import roslib; 
import rospy
import numpy as np
import os
from std_msgs.msg import String, Bool

# The laser scan message from Kinect
from sensor_msgs.msg import LaserScan

# The velocity command message from Kobuki
from geometry_msgs.msg import Twist

# We use a hyperbolic tangent as a transfer function (controls based on previous works found on the Internet)
from math import tanh

# This class will follow the nearest thing to it, with the hypothesis that it is a human
class follower:
    def __init__(self, followDistance=1.2, stopDistance=1.0, max_speed=0.4, min_speed=0.01 ):
        # Subscribe to the laser scan data
        self.sub = rospy.Subscriber('scan', LaserScan, self.laser_callback)

        # Publish movement commands to the turtlebot's base
        self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist)

	# Subscribe to the voice control messages
	self.sub2 = rospy.Subscriber('follow', String, self.voice_callback, None, 1)

	# Subscribe to tracking to verify if the detected object is a person, so it will be followed
	#self.sub3 = rospy.Subscriber('tracking', Bool, tracker_callback)

        # Define the minimum stop distance to followed human, speed limits and distance to keep when moving
        self.stopDistance = stopDistance
        self.max_speed = max_speed
        self.min_speed = min_speed
	self.followDist = followDistance

	# Variables to be read from measurements: the distance to the closest object, and its position in array, respectively
	self.closest = 0
	self.position = 0

        # Create a Twist message, and set initial values to be sure the robot will start stopped
        self.command = Twist()
        self.command.linear.x = 0.0
        self.command.linear.y = 0.0
        self.command.linear.z = 0.0
        self.command.angular.x = 0.0
        self.command.angular.y = 0.0
        self.command.angular.z = 0.0

	# Follow flag variable to decide when to start/stop following
	self.followFlag = True

    def laser_callback(self, scan):
	# Determines the closest thing to the Robot	
	self.getPosition(scan)
	# DEBUG MSG: 
	#rospy.logdebug('position: {0}'.format(self.position))

	# If there's something within the acceptable following distance, the robot starts following
	if (self.closest < self.followDist and self.followFlag == True):
	    self.follow()

	# When following, it publishes to Bark.py if the human comes closer to the robot
	    if (self.closest < 0.9*self.stopDistance and self.closest > 0.8*self.stopDistance and self.followFlag == True):
	    	self.pubbark = rospy.Publisher('follow', String)
		self.pubbark.publish(String("Bark"))
		self.pubbark.publish(String("STOPBark"))
	# Two distance ranges are defined and the sound reproduced will be different for each range 
	    elif (self.closest < 0.8*self.stopDistance and self.closest > 0.7*self.stopDistance and self.followFlag == True):
	    	self.pubbark = rospy.Publisher('follow', String)
		self.pubbark.publish(String("Bark2"))
		self.pubbark.publish(String("STOPBark"))

	# Else, since the distance between the human and the robot does not change, sit and wait
	else:
	    self.stop() 

        # DEBUG MSG:
        #rospy.logdebug('Distance: {0}, speed: {1}, angular: {2}'.format(self.closest, self.command.linear.x, self.command.angular.z))
	
	# Ensure we have only one publish command with the Twist message defined before
	self.pub.publish(self.command)

	# Treats the publications of the voice control node to start/stop following people and barking
    def voice_callback(self, data):
	if str(data) == "data: Stop":
	    self.followFlag = False
            print self.followFlag
	    self.stop()

	elif str(data) == "data: Follow":
	    self.followFlag = True
            print self.followFlag
	    self.follow()

	# Verifies if the detected object is a person to start following
    def tracker_callback(data):
	if str(data) == "data: True":
	    self.followFlag = True

	# Starts following the nearest object: speeds are controled based on the distance error (targeting the stopDistance)
    def follow(self):
	self.command.linear.x = tanh(5 * (self.closest - self.stopDistance)) * self.max_speed
	# Turns faster the further he is turned from the intended object (empiric values)
	self.command.angular.z = ((self.position-320.0)/320.0)
	
	# If it is going slower than our min_speed, just stop (DEBUG)
	if abs(self.command.linear.x) < self.min_speed:
	    self.command.linear.x = 0.0  

	# Stop moving function
    def stop(self):
        self.command.linear.x = 0.0
	self.command.angular.z = 0.0

	# Function to read and allocate self.closest and self.position
    def getPosition(self, scan):
        # Build a depths array to filter nan data from scan.ranges
	depths = []
	for dist in scan.ranges:
	    if not np.isnan(dist):
			depths.append(dist)
	# Turns a tupple into an array
	fullDepthsArray = scan.ranges[:]

	# If depths is empty that means the robot is too close to the object to get a measurement
	if len(depths) == 0:
	    self.closest = 0
	    self.position = 0
	# Then establish our distance/position to nearest object as "0" to the referential
	else:
	    self.closest = min(depths)
	    self.position = fullDepthsArray.index(self.closest)
	
def listener():
	print "I am listening"
	# We were trying to create a random navigation to the robot, but we were not able to finish it on time	
	#rospy.Subscriber("move", Bool, callback)
	#rospy.spin()

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('follow', log_level=rospy.DEBUG, anonymous=True)
    listener()
    follower = follower()
    rospy.spin()

