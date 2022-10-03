#!/usr/bin/env python

import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

global X, Y, th, V, W
X = 0.0
Y = 0.0
th = 0.0
V = 0.0
W = 0.0

global des_vel
des_vel = Twist()

pub = rospy.Publisher('/pioneer/cmd_vel', Twist, queue_size=100)

def pose_callback(msg):
    global X, Y
    X = msg.pose.pose.position.x
    Y = msg.pose.pose.position.y

    q0 = msg.pose.pose.orientation.w
    q1 = msg.pose.pose.orientation.x
    q2 = msg.pose.pose.orientation.y
    q3 = msg.pose.pose.orientation.z

    V = msg.twist.twist.linear.x
    W = msg.twist.twist.linear.z


    global th
    th = math.atan2(2*(q0*q3 + q1*q2),1-2*((q2)**2+(q3)**2))

    print( X, Y, th*180/(math.pi))
    
    global des_vel
    des_vel.linear.x = 0.0
    des_vel.angular.z = 0.0
    pub.publish(des_vel)

def main():
    rospy.init_node('p3dx_pub_N_sub', anonymous=True)
    rospy.Subscriber('/odom', Odometry, pose_callback)
    rospy.spin()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


