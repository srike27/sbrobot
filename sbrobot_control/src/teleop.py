#!/usr/bin/env python
# Initial code created by Graylin Trevor Jay (tjay@cs.brown.edu) an published under Crative Commens Attribution license.
# addition for signal interrupt by Koen Buys

#import youbot_driver_ros_interface
#import roslib; roslib.load_manifest('youbot_oodl')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

import sys, select, termios, tty, signal

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   q    w    e
   1    k    3
   a    x    d

y/n : increase/decrease max speeds by 10%
t/b : increase/decrease only linear speed by 10%
r/v : increase/decrease only angular speed by 10%
u/m : tilt camera angle up/down
anything else : stop

CTRL-C to quit
"""

moveBindings = {
#x,y,tetha ratio
		'w':(1,0,0), 	# forwards
		'e':(1,0,-1), 	# forwards + rotation right
		'1':(0,1,0), 	# left
		'3':(0,-1,0),	# right
		'q':(1,0,1), 	# forwards + rotation left
		'x':(-1,0,0), 	# backward
		'd':(0,0,-1), 	# turn right on spot
		'a':(0,0,1), 	# turn left on spot
	       }

speedBindings={
		'y':(1.2,1.2),
		'n':(.8,.8),
		't':(1.2,1),
		'b':(.8,1),
		'r':(1,1.5),
		'v':(1,.5),
	      }
servoBindings={'u':(1),'m':(-1)}



class TimeoutException(Exception): 
    pass 

def getKey():
    def timeout_handler(signum, frame):
        raise TimeoutException()
    
    old_handler = signal.signal(signal.SIGALRM, timeout_handler)
    signal.alarm(1) #this is the watchdog timing
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    try:
       key = sys.stdin.read(1)
       #print "Read key"
    except TimeoutException:
       #print "Timeout"
       return "-"
    finally:
       signal.signal(signal.SIGALRM, old_handler)

    signal.alarm(0)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = 0.1
turn = 0.3

def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('cmd_vel', Twist)
	servop = rospy.Publisher('camera_angle_controller/command',Float64)
	rospy.init_node('teleop_twist_keyboard')
	servo = Float64()
	servo.data = 0
	x = 0
	y = 0
	th = 0
	status = 0

	try:
		print msg
		print vels(speed,turn)
		while(1):
			key = getKey()
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				y = moveBindings[key][1]
				th = moveBindings[key][2]
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]

				print vels(speed,turn)
				if (status == 14):
					print msg
				status = (status + 1) % 15
			elif key in servoBindings.keys():
				servo.data += 0.1*servoBindings[key]
			else:
				x = 0
				y = 0
				th = 0
				if (key == '\x03'):
					break

			twist = Twist()
			twist.linear.x = x*speed 
			twist.linear.y = y*speed 
			twist.linear.z = 0

			twist.angular.x = 0 
			twist.angular.y = 0
			twist.angular.z = th*turn
			pub.publish(twist)
			servop.publish(servo)

	except:
		print e

	finally:
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)
		servoo = Float64()
		servoo.data = 0
		servop.publish(servoo)
    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)



