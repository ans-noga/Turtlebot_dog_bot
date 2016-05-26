#!/usr/bin/env python

# Imports ROS basic functions and 
import sys
import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String

# If Follow.py publishes "Bark" messages, then this node will play the sound

def callback(data):
	# To be sure it will bark only once, a StopBarking message was added
	if str(data) == "data: STOPBark":
		soundhandle = SoundClient()
		soundhandle.stopAll()
	# The first bark sound is played if the human approaches the robot a little further than the stopDistance 
	elif str(data) == "data: Bark":
		# Creates an instance of SoundClient
		soundhandle = SoundClient()
		#soundhandle.play(1) (DEBUG)
		soundhandle.playWave('/home/turtlebot/catkin_ws/src/maximum_bot/scripts/bark.wav')
		pubshoot = rospy.Publisher('bark', String) # (DEBUG)
		# Introduce a delay to be sure the whole sound will be played
		rospy.sleep(2)
	# The second bark sound is played if the human approaches the robot a a lot further than the stopDistance 
	elif str(data) == "data: Bark2":
		# Creates an instance of SoundClient
		soundhandle = SoundClient()
		#soundhandle.play(1) (DEBUG)
		soundhandle.playWave('/home/turtlebot/catkin_ws/src/maximum_bot/scripts/scream.wav')
		pubshoot = rospy.Publisher('bark', String) # (DEBUG)
		# Introduce a delay to be sure the whole sound will be played
		rospy.sleep(2)

# Subscribes to Follow.py
def listener():
	print "I am listening"
	rospy.init_node('bark', anonymous=True)
	# Be sure to bark only once even if several messages are published in a row
	rospy.Subscriber("follow", String, callback, None, 1)
	rospy.spin()

if __name__ == '__main__':
	listener()
