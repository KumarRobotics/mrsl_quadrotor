#!/usr/bin/env python2
# license removed for brevity
import rospy
from quadrotor_msgs.msg import SO3Command

def talker():
    pub = rospy.Publisher('so3_cmd', SO3Command, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    so3_cmd = SO3Command();
    while not rospy.is_shutdown():
        so3_cmd.force.x = 0 
        so3_cmd.force.y = 0 
        so3_cmd.force.z = 0.5*9.9
        so3_cmd.orientation.w = 1 
        so3_cmd.orientation.x = 0
        so3_cmd.orientation.y = 0
        so3_cmd.orientation.z = 0
        so3_cmd.kR = [1.0, 1.0, 1.0] 
        so3_cmd.kOm = [0.1, 0.1, 0.1] 
        so3_cmd.aux.kf_correction = 0
        rospy.loginfo(so3_cmd)
        pub.publish(so3_cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

