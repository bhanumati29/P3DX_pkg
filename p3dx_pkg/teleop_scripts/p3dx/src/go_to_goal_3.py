#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

x0 = -2
y0 = 1
th0 = 180

kw = 0.5/15
kx = 0.2/1
ky = 0.2/1

x   = 0
y   = 0
th  = 0

vx  = 0
vy  = 0
vth = 0

def odom_cb(msg, twist_pub):
	global x
	global y
	global th
	global vx
	global vy
	global vth

	x   = msg.pose.pose.position.x
	y   = msg.pose.pose.position.y

	q0 = msg.pose.pose.orientation.w
	q1 = msg.pose.pose.orientation.x
	q2 = msg.pose.pose.orientation.y
	q3 = msg.pose.pose.orientation.z
	th = math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))*180/math.pi

	vx  = msg.twist.twist.linear.x
	vy  = msg.twist.twist.linear.y
	vth = msg.twist.twist.angular.z

if __name__ == '__main__':
	rospy.init_node('go_to_goal')
	twist_pub = rospy.Publisher('RosAria/cmd_vel', Twist, queue_size=1)
	rospy.Subscriber('/RosAria/pose', Odometry, odom_cb, twist_pub)
	#twist_pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1)
	#rospy.Subscriber('turtle1/pose', Odometry, odom_cb, twist_pub)

	desired_pose = Odometry()
	desired_twist = Twist()

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		th1 = th

		distance = math.sqrt((x0-x)*(x0-x) + (y0-y)*(y0-y))

		if (distance<0.075):
			th2 = th0
		else:
			th2 = math.atan2(y0-y, x0-x)*180/math.pi
		e_th = th2-th1

		if (e_th>180):
			e_th = e_th-360
		if (e_th<=-180):
			e_th = e_th+360

		if (distance>0.075):
			if(e_th>-180 and e_th<-90):
				w0 = kw*(e_th+180)
			if(e_th>=-90 and e_th<=90):
				w0 = kw*(e_th)
			if(e_th>90 and e_th<=180):
				w0 = kw*(e_th-180)
		else:
			w0 = kw*(e_th)

		if (w0>0.5):
			w0 = 0.5
		if (w0<-0.5):
			w0 = -0.5

		if (distance<0.075):
			v0 = 0
		else:
			v0 = math.sqrt(kx*kx*(x0-x)*(x0-x) + ky*ky*(y0-y)*(y0-y))*math.cos(e_th*math.pi/180)

		if (v0>0.2):
			v0 = 0.2
		if (v0<-0.2):
			v0 = -0.2

		desired_twist.linear.x  = v0
		desired_twist.angular.z = w0
		twist_pub.publish(desired_twist)
		print th
		rate.sleep()
