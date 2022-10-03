#!/usr/bin/env python

import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time


global strt, t_init, t_now
strt = True
t_init = 0.0
t_now = 0.0

global X, Y, th, V, W
X = 0.0
Y = 0.0
th = 0.0
V = 0.0
W = 0.0


global des_vel
des_vel = Twist()

pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=100)

def get_traj_input(t):
    x_traj = -1.0
    y_traj = 0.0

    ''' T = 50.0
    freq = 1.0/T
    omega = 2.0*math.pi*freq
    Ax = 0.75
    Ay = 0.50
    
    x_traj = Ax*math.sin(omega*t)
    y_traj = Ay*math.sin(2.0*omega*t)'''

    return x_traj, y_traj

def limit(x,x_min,x_max):
    if(x<x_min):
        return x_min
    elif(x>x_max):
        return x_max
    else:
        return x

def wrap_pi(b_):
    if (b_ <= -math.pi):
        while(b_ <= -math.pi):
            b_ = b_ + 2.0*math.pi
    elif (b_ > math.pi):
        while(b_ > math.pi):
            b_ = b_ - 2.0*math.pi
    #print(b_*180.0/math.pi)
    return b_

def wrap_pi_2(b_):
    if (b_ <= -math.pi/2.0):
        while(b_ <= -math.pi/2.0):
            b_ = b_ + math.pi
    elif (b_ > math.pi/2.0):
        while(b_ > math.pi/2.0):
            b_ = b_ - math.pi
    return b_

def a2b(X_, Y_, th_, x0_, y0_):
    k1 = 0.15
    k2 = 2/(math.pi)*k1
    d  = math.sqrt((x0_- X_)**2 + (y0_- Y_)**2)
    V_ = k1 * d
    alpha = math.atan2((y0_- Y_), (x0_- X_))
    b = alpha - th_
    beta = wrap_pi(b)

    W_= k2 * beta

    return V_, W_

def a2b2(X_, Y_, th_, x0_, y0_):
    k1 = 0.15
    k2 = 0.75
    d  = math.sqrt((x0_- X_)**2 + (y0_- Y_)**2)
    V_ = k1 * d
    alpha = math.atan2((y0_- Y_), (x0_- X_))
    b = alpha - th_
    beta = wrap_pi(b)
    dirn_frwrd = (beta> -math.pi/2.0) and (beta<= math.pi/2.0) 
    beta = wrap_pi_2(b)
    W_= k2 * beta
    if(dirn_frwrd):
        return V_, W_
    else:
        return -V_, W_

    return V_, W_

def a2b3(X_, Y_, th_, x0_, y0_, th_0_):
    k1 = 0.15
    k2 = 0.75
    k3 = 1.0

    d  = math.sqrt((x0_- X_)**2 + (y0_- Y_)**2)
    V_ = k1 * d
    alpha = math.atan2((y0_- Y_), (x0_- X_))
    b = alpha - th_
    beta = wrap_pi(b)
    dirn_frwrd = (beta> -math.pi/2.0) and (beta<= math.pi/2.0) 
    beta = wrap_pi_2(b)
    gamma = wrap_pi(th_0_ - th_)

    W_= k2 * beta + k3 *limit((1.0/d), -1.0, 1.0 )* gamma

    print(beta*180.0/(math.pi), gamma*180.0/(math.pi))

    if(dirn_frwrd):
        return V_, W_
    else:
        return -V_, W_
    

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

    #print( X, Y, th*180/(math.pi))
    
    global des_vel
    des_vel.linear.x = 0.0
    des_vel.angular.z = 0.0

    global strt
    if(strt==True):
        global strt, t_init
        strt = False
        t_init = time.time()
    t_now = time.time() - t_init

    # Goal position
    x0 = 1.0
    y0 = 0.0
    th_0 = 45.0 * math.pi/180.0

    # x0, y0 = get_traj_input(t_now)
    
    des_V, des_W = a2b3(X, Y, th, x0, y0, th_0)
    
    v_limit = 1.2
    w_limit = 300.0 * (math.pi)/180.0
    des_V = limit(des_V, -v_limit, v_limit)
    des_W = limit(des_W, -w_limit, w_limit)

    #print(des_V, des_W)

    des_vel.linear.x  = des_V 
    des_vel.angular.z = des_W 

    pub.publish(des_vel)

def main():
    rospy.init_node('p3dx_pub_N_sub', anonymous=True)
    rospy.Subscriber('/RosAria/pose', Odometry, pose_callback)
    rospy.spin()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


