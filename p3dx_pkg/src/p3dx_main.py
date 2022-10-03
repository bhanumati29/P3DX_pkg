#!/usr/bin/env python

import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
import time
import art_pot_field as ap
import numpy as np
import path_tracking_algo as tracker
import path_2D as path

global strt, t_init, t_now
strt = True
t_init = 0.0
t_now = 0.0

global des_vel
des_vel = Twist()

global X, Y, th, V, W
X = 0.0
Y = 0.0
th = 0.0
V = 0.0
W = 0.0

global X_v, Y_v, th_v
X_v  = 0.0
Y_v  = 0.0
th_v = 0.0

pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=100)

def get_traj_input(t):
    x_traj = 2.0
    y_traj = 0.0

    # T = 50.0
    # freq = 1.0/T
    # omega = 2.0*math.pi*freq
    # Ax = 0.75
    # Ay = 0.50
    
    # x_traj = Ax*math.sin(omega*t)
    # y_traj = Ay*math.sin(2.0*omega*t)

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

global final_goal_reached
final_goal_reached = False

def a2b3(X_, Y_, th_, x0_, y0_, th_0_):
    # print(X_,Y_)
    k1 = 0.15
    k2 = 0.75
    k3 = 1.0

    d  = math.sqrt((x0_- X_)**2 + (y0_- Y_)**2)

    global final_goal_reached
    final_goal_reached = d < 0.075

    if(final_goal_reached==False):
        V_ = k1 * d
        alpha = math.atan2((y0_- Y_), (x0_- X_))
        b = alpha - th_
        beta = wrap_pi(b)
        dirn_frwrd = (beta> -math.pi/2.0) and (beta<= math.pi/2.0) 
        beta = wrap_pi_2(b)
        W_= k2*beta
        print('------------------------')

        if(dirn_frwrd):
            return V_, W_
        else:
            return -V_, W_

        # print(beta*180.0/(math.pi), gamma*180.0/(math.pi))
    else:
        print('=========================')
        global final_goal_reached
        final_goal_reached = True
        V_ = 0.0
        W_ = k2*wrap_pi(th_0_ - th_)
        print(V_, W_)
        return V_, W_
    
def pose_callback(msg):
    global X, Y, V, W
    X = msg.pose.pose.position.x
    Y = msg.pose.pose.position.y

    q0 = msg.pose.pose.orientation.w
    q1 = msg.pose.pose.orientation.x
    q2 = msg.pose.pose.orientation.y
    q3 = msg.pose.pose.orientation.z

    V = msg.twist.twist.linear.x
    W = msg.twist.twist.angular.z
    
    # print(V,W*180.0/math.pi)

    global des_vel
    des_vel.linear.x = 0.0
    des_vel.angular.z = 0.0

    global strt
    if(strt==True):
        global strt, t_init
        strt = False
        t_init = time.time()
    t_now = time.time() - t_init

    th_0 = 45.0 * math.pi/180.0

    x0, y0 = get_traj_input(t_now)


    global X_v, Y_v, th_v
    
    #### Go to goal starts
    # des_V, des_W = a2b3(X_v, Y_v, th_v, x0, y0, th_0)
    #### Go to goal ends

    #### Artificial potential field starts
    # f_ap = ap.f_total(X_v, Y_v, x0, y0, -0.3, 0.2, 0.5, 1.2)
    # xy_gd = ap.get_new_xy(np.array([X_v, Y_v]),f_ap,1.0)
    # des_V, des_W = a2b3(X_v, Y_v, th_v, xy_gd[0], xy_gd[1], th_0)
    # print([X_v,Y_v],xy_gd,des_V,des_W)
    #### Artificial potential field ends

    #### Pure pursuit algorithm starts
    v = 0.1 # 0.15
    l_d = 0.7 #0.20
    x_traj = [-1.5]
    y_traj = [0]
    # x_n = np.linspace(-2.0,2.0,20)
    # y_n = np.sin(x_n)
    # x_traj = np.asarray(x_n)
    # y_traj = np.asarray(y_n)
    # print(x_traj, y_traj)
    
    x_traj, y_traj = path.get_st_line()
    x1,y1,x2,y2,n_switch, goal_reached = tracker.get_xy_12(x_traj, y_traj, X_v, Y_v, l_d)
    print(x1,y1,x2,y2,n_switch,goal_reached)
    th_0 = -90.0*math.pi/180.0

    x_g, y_g = 0.0, 0.0
    if(goal_reached==True):
        x_g, y_g = x_traj[len(x_traj)-1], y_traj[len(y_traj)-1]
    else:  
        if(n_switch!=0):
            x_g, y_g = tracker.get_x_g_y_g(X_v, Y_v, l_d, x1, y1, x2, y2)
        else:
            x_g, y_g = x1, y1
    
    # des_W = tracker.pure_pursuit(v, l_d, th_v, X_v, Y_v, x_g, y_g)
    # des_V = v
    
    des_V, des_W = 0, 0
    if(goal_reached==False):
        global final_goal_reached
        final_goal_reached = False
        des_W = tracker.pure_pursuit(v, l_d, th_v, X_v, Y_v, x_g, y_g)
        des_V = v
    else:
        des_V, des_W = a2b3(X_v, Y_v, th_v, x_g, y_g, th_0)
    #### Pure pursuit algorithm ends

    v_limit = 1.2
    w_limit = 300.0 * (math.pi)/180.0
    des_V = limit(des_V, -v_limit, v_limit)
    des_W = limit(des_W, -w_limit, w_limit)

    # print(des_V, des_W)

    # des_V, des_W =  0, 0*math.pi/180.0

    des_vel.linear.x  = des_V 
    des_vel.angular.z = des_W 

    pub.publish(des_vel)

def vicon_callback(msg2):
    global X_v, Y_v, th_v
    X_v = msg2.transform.translation.x
    Y_v = msg2.transform.translation.y
    Z_v = msg2.transform.translation.z

    q0_v = msg2.transform.rotation.w
    q1_v = msg2.transform.rotation.x
    q2_v = msg2.transform.rotation.y
    q3_v = msg2.transform.rotation.z

    th_v = math.atan2(2*(q0_v*q3_v + q1_v*q2_v),1-2*((q2_v)**2+(q3_v)**2))

    # print(round(X_v,3), round(Y_v,3), round(th_v*180.0/math.pi,3))

def main():
    rospy.init_node('p3dx_main', anonymous=True)
    rospy.Subscriber('/RosAria/pose', Odometry, pose_callback)
    rospy.Subscriber('/vicon/p3dx_vicon_pose/p3dx_vicon_pose', TransformStamped, vicon_callback)
    rospy.spin()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
