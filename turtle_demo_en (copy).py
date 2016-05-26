#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from geometry_msgs.msg import Twist

from rospeex_if import ROSpeexInterface
import re

rospeex = None
turtle_pub = None

OPERATION_TIME = 500
TURN_TIME = 11
TURN_ACTION_COUNTER_MAX = 7
TARGET_SPEED = 0.12
TARGET_TURN = 1.0

_target_speed = 0.0
_control_speed = 0.0
_target_turn = 0.0
_control_turn = 0.0
_operation_count = 0
_turn_action = 0
_started = False
_force_stop = False
_force_stop = False
_during_right = False
_during_left = False

_pattern_rule = ""

def set_ope_param( target_speed, target_turn ):
    global _target_speed, _target_turn, _operation_count
    _target_speed = target_speed
    _target_turn = target_turn
    _operation_count = 0

def speak(msg):
     rospeex.say( msg, language='en', engine='nict', voice_font='EF007')

def timer_callback(event):
    global _target_speed, _control_speed, _target_turn, _control_turn, _operation_count, _during_right, _during_left, _started, _force_stop, _turn_action, OPERATION_TIME, TARGET_SPEED, TURN_TIME, TURN_ACTION_COUNTER_MAX
    _operation_count += 1
    if _force_stop == True:
        _started = False
        _force_stop = False
        _during_right = False
        _during_left = False
        _turn_action = 0
    elif _during_right == True:
        if _operation_count > TURN_TIME:
            _turn_action += 1
            if _turn_action == TURN_ACTION_COUNTER_MAX:
                set_ope_param(0, TARGET_TURN)
            else:
                set_ope_param(TARGET_SPEED, 0)
    elif _during_left == True:
        if _operation_count > TURN_TIME:
            _turn_action += 1
            if _turn_action == TURN_ACTION_COUNTER_MAX:
                set_ope_param(0, -TARGET_TURN)
            else:
                set_ope_param(TARGET_SPEED, 0)
    elif _operation_count > OPERATION_TIME and _started == True:
        speak( "It's time over.")
        print "It's time over."
        set_ope_param( 0, 0 )
        _started = False

    turtle_move( _target_speed, _target_turn )
    if _turn_action > TURN_ACTION_COUNTER_MAX:
        _turn_action = 0
        _during_right = False
        _during_left = False

def sr_response(message):
    rospy.loginfo(message)
    global TARGET_SPEED, TARGET_TURN, _during_right, _during_left, _started, _force_stop, _pattern_rule
    print 'you said : \"%s\"' %message

    m = _pattern_rule.match(message.lower())

    if m is not None:
        operation = m.group(2)
        print "recognized \"%s\"" %operation

        if ( operation == "back" ) and _started == False:
            speak("I'm moving backward.")
            _started = True
            set_ope_param(-TARGET_SPEED, 0)

        elif operation == "come" and _started == False:
            print "Following."
            _started = True
            set_ope_param(TARGET_SPEED*0.08, 0)

        elif operation == "start" or operation == "straight":
            speak("I'm starting.")
            print "I'm starting."
            _started = True
            set_ope_param(TARGET_SPEED, 0)

        elif operation == "right":
            if _during_right == True:
                speak("I'm already turning.")
                print "I'm already turning."
            elif _started == True:
                _during_right = True
                speak("I'm turning right.")
                print "I'm turning right."
                set_ope_param(0, -TARGET_TURN)

        elif operation == "left":
            if _during_left == True:
                speak("I'm already turning.")
                print "I'm already turning."
            elif _started == True:
                _during_left = True
                speak("I'm turning left.")
                print "I'm turning left."
                set_ope_param(0, TARGET_TURN)

        elif operation == "stop" or operation == 'wait':
            speak("I stopped.")
            print "I stopped."
            _force_stop = True
            set_ope_param(0, 0)

    elif message != "":
        print "What?"
        rospeex.play_sound( "alert.wav" )

def turtle_move( target_speed, target_turn ):
    global _control_speed, _control_turn

    if target_speed > _control_speed:
        _control_speed = min( target_speed, _control_speed + 0.02 )
    elif target_speed < _control_speed:
        _control_speed = max( target_speed, _control_speed - 0.02 )
    else:
        _control_speed = target_speed

    if target_turn > _control_turn:
        _control_turn = min( target_turn, _control_turn + 0.1 )
    elif target_turn < _control_turn:
        _control_turn = max( target_turn, _control_turn - 0.1 )
    else:
        _control_turn = target_turn

    control_speed = _control_speed
    control_turn = _control_turn

    twist = Twist()
    twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
    turtle_pub.publish(twist)

if __name__=="__main__":

    _pattern_rule = re.compile( '.*(robot)*.*(back|come|start|straight|right|left|stop|wait)')

    _operation_count = 0
    _turn_action = 0
    _started = False
    _force_stop = False
    _force_stop = False
    _during_right = False
    _during_left = False
    rospy.init_node('rospeex_turtle')
    rospeex = ROSpeexInterface()
    rospeex.init()
    rospeex.register_sr_response( sr_response )
    rospeex.set_spi_config(language='en',engine='nict')
    #rospeex.set_spi_config(language='en',engine='google')
    turtle_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist)
    rospy.Timer(rospy.Duration(0,100000000), timer_callback)

    try:
        rospy.spin();
    except:
        print e
    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        turtle_pub.publish(twist)
