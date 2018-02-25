#!/usr/bin/python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

twist_pub = rospy.Publisher('vel_cmd', Twist, queue_size=1)
deadzone = 0.5

def callback(data):
    vx = data.axes[1]
    # if inside deadzone, set the value to zero 
    if vx < deadzone and vx > -deadzone:
        vx = 0
    elif vx >= deadzone:
        vx = (vx - deadzone) / (1 - deadzone) * 2
    else:
        vx = (vx + deadzone) / (1 - deadzone) * 2
    # if inside deadzone, set the value to zero 
    vy = data.axes[0] 
    if vy < deadzone and vy > -deadzone:
        vy = 0
    elif vy >= deadzone:
        vy = (vy - deadzone) / (1 - deadzone) * 2
    else:
        vy = (vy + deadzone) / (1 - deadzone) * 2
  
    vz = data.axes[4] 
    # if inside deadzone, set the value to zero 
    if vz < deadzone and vz > -deadzone:
        vz = 0
    elif vz >= deadzone:
        vz = (vz - deadzone) / (1 - deadzone) 
    else:
        vz = (vz + deadzone) / (1 - deadzone) 
  
    w = data.axes[3]
    # if inside deadzone, set the value to zero 
    if w < deadzone and w > -deadzone:
        w = 0
    elif w >= deadzone:
        w = (w - deadzone) / (1 - deadzone) 
    else:
        w = (w + deadzone) / (1 - deadzone) 
  
    cmd = Twist()
    cmd.linear.x = vx
    cmd.linear.y = vy
    cmd.linear.z = vz
    cmd.angular.z = w

    twist_pub.publish(cmd)

def listener():
    rospy.init_node('joy_to_twist')
    rospy.Subscriber("joy", Joy, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

            
if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
