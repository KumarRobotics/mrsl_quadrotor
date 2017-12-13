#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty, rospy
import curses

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving options:
---------------------------
   w -- up (+z)
   s -- down (-z)
   a -- counter clockwise yaw
   d -- clockwise yaw
   up arrow -- forward (+x)
   down arrow -- backward (-x)
   <- -- forward (+y)
   -> -- backward (-y)
   CTRL-C to quit
"""
print msg
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.2)
    if rlist:
        key = sys.stdin.read(1)
        ### if using arrow keys, need to retrieve 3 keys in buffer
	if ord(key) == 27:
            key = sys.stdin.read(1)
	    if ord(key) == 91:
               key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)

	rospy.init_node('teleop_twist_keyboard')
	pub = rospy.Publisher('~cmd_vel', Twist, queue_size = 1)

	v = rospy.get_param("~v", 2.0)
	w = rospy.get_param("~w", 1.0)

        rate = rospy.Rate(20) # 10hz


       	while not rospy.is_shutdown():
            	vx = 0
        	vy = 0
        	vz = 0
        	wy = 0
       		key = getKey()
		if key == 'w':
			vx = v
		elif key == 's':
			vx = -v
		elif key == 'a':
			vy = v
		elif key == 'd':
			vy = -v
		elif key=='A':
			vz = v
		elif key=='B':
			vz = -v
		elif key=='C':
			wy = -w
		elif key=='D':
			wy = w
       		if (key == '\x03'):
       			break
       		twist = Twist()
       		twist.linear.x = vx; twist.linear.y = vy; twist.linear.z = vz;
       		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = wy
       		pub.publish(twist)
       		rate.sleep()


