#!/usr/bin/env python

# Import ROS basic functions and Python's Transform library for frame trasformations
import rospy
import tf
from std_msgs.msg import Bool

def tracker():
    # Checks to see if what the robot detects is a person or not, then if it is
    # it will publish True to follow.py, else it will publish False."""
    rospy.init_node('tracking')
    pub = rospy.Publisher('tracking', Bool)
    while not rospy.is_shutdown():
        t = tf.TransformListener()
        if t.frameExists('openni_depth_frame'):
            track = True
        else:
            track = False
        rospy.loginfo("list: %s", t.getFrameStrings())
        pub.publish(track)
        rospy.sleep(1.0)

if __name__ == '__main__':
	tracker()

