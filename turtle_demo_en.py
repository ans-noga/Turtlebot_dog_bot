#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from geometry_msgs.msg import Twist

from rospeex_if import ROSpeexInterface
import re
from std_msgs.msg import String

rospeex = None
turtle_pub = None

_pattern_rule = ""

def speak(msg):
     rospeex.say( msg, language='en', engine='nict', voice_font='EF007')

def sr_response(message):
    rospy.loginfo(message)
    global _started, _force_stop, _pattern_rule
    print 'you said : \"%s\"' %message

    m = _pattern_rule.match(message.lower())

    if m is not None:
        operation = m.group(2)
        print "recognized \"%s\"" %operation

        if operation == "come" or operation == "follow" or operation == "on" or operation == "one" or operation == "start":
	    pubvoice = rospy.Publisher('follow', String)
            pubvoice.publish(String("Follow"))            
	    print "Following. and published"
	    speak("WoofWoof")

        elif operation == "stop" or operation == "wait" or operation == "off" or operation == "two":
	    pubvoice = rospy.Publisher('follow', String)
            pubvoice.publish(String("Stop"))
            print "I stopped. and published"
	    speak("WoofWoof")

    elif message != "":
	#speak("What")
        print "What?"

if __name__=="__main__":

    _pattern_rule = re.compile( '.*(robot)*.*(back|come|stop|wait)')

    rospy.init_node('rospeex_turtle')
    rospeex = ROSpeexInterface()
    rospeex.init()
    rospeex.register_sr_response( sr_response )
    rospeex.set_spi_config(language='en',engine='nict')

    try:
        rospy.spin();
    except:
        print e
