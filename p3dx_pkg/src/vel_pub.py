#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist


global msg
msg = Twist()

def pub_vel():
    rospy.init_node('pub_vel', anonymous=True)
    pub = rospy.Publisher('/pioneer/cmd_vel', Twist, queue_size=100)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        global msg
        msg.linear.x = 0.0
        msg.angular.z = 0.0

        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__=='__main__':
    try:
        pub_vel()
    except rospy.ROSInterruptException:
        pass
