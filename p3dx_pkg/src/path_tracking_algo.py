import math

def pure_pursuit(v, l_d, th, x, y, x_g, y_g):
    alpha = math.atan2(y_g - y, x_g - x) - th
    w = (2.0*v/l_d)*math.sin(alpha)
    return w

def get_x_g_y_g(x, y, L, x1, y1, x2, y2):
    x_g = 0
    y_g = 0

    a = x1 - x
    b = x2 - x1
    c = y1 - y
    d = y2 - y1

    k1 = b**2 + d**2
    k2 = 2.0*(a*b + c*d)
    k3 = a**2 + c**2 - L**2

    lam_1 = (-k2 - math.sqrt(k2**2 - 4.0*k1*k3))/(2.0*k1)
    lam_2 = (-k2 + math.sqrt(k2**2 - 4.0*k1*k3))/(2.0*k1)

    lam = 0.0
    num_sol = 0
    lam_1_feas = False
    lam_2_feas = False

    if((lam_1>=0.0) and (lam_1<=1.0)):
        lam_1_feas = True
        num_sol += 1
    
    if((lam_2>=0.0) and (lam_2<=1.0)):
        lam_2_feas = True
        num_sol += 1

    if (num_sol == 2):
        lam = max(lam_1, lam_2)
    elif(num_sol == 1):
        if(lam_1_feas==True):
            lam = lam_1
        else:
            lam = lam_2
    else:
        pass
    
    x_g = x1 + lam*(x2 - x1)
    y_g = y1 + lam*(y2 - y1)
    
    return x_g, y_g

x, y, L, x1, y1, x2, y2 = 0.0, 0.0, 2.0, 5.0, 1.0, 0, 1.0
x_g, y_g = get_x_g_y_g(x, y, L, x1, y1, x2, y2)
# print(x_g, y_g)                                      


def get_xy_12(x_traj, y_traj, x, y, l_d):
    goal_reached = False
    d_arr = []
    d_in_arr = []
    n = len(x_traj)
    for i in range(0,n):
        d_i = math.sqrt((x - x_traj[i])**2 + (y - y_traj[i])**2)
        d_arr.append(d_i)
        if(d_i<=l_d):
            d_in_arr.append(True)
        else:
            d_in_arr.append(False)
    
    n_switch = 0
    index_switch = []
    for i in range(1,n):
        if (d_in_arr[i] != d_in_arr[i-1]):
            n_switch += 1
            index_switch.append(i-1)
    
    x1,y1,x2,y2 = 0,0,0,0

    if (n_switch==0):
        i = 0
        d_min = 9999999.9
        for j in range(0,n):
            if(d_arr[j]<=d_min):
                i = j
                d_min = d_arr[j]
        if(i!=n-1):
            x1 = x_traj[i]
            y1 = y_traj[i]
            x2 = x_traj[i+1]
            y2 = y_traj[i+1]
        else:
            # print('goalreached!------------')
            pass
    elif (n_switch==1):
        if (d_in_arr[0] ==True):
            i = index_switch[0]
            x1 = x_traj[i]
            y1 = y_traj[i]
            x2 = x_traj[i+1]
            y2 = y_traj[i+1]
        else:
            # print('goalreached!')
            goal_reached = True
    else:
        i = index_switch[1]
        x1 = x_traj[i]
        y1 = y_traj[i]
        x2 = x_traj[i+1]
        y2 = y_traj[i+1]
    return x1,y1,x2,y2, n_switch, goal_reached

