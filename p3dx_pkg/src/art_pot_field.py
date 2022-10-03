import numpy as np
import math
import matplotlib.pyplot as plt

x = -1.0
y = 0.0

x_f = 2.0
y_f = 0.0

x_o = -0.3
y_o = 0.2

d_0 = 0.5
rho_0 = 1.0

zeta = 0.7
eta = 0.2

def f_att(x, y, x_f, y_f, d_0):
    P = np.array([x, y])
    P_f = np.array([x_f, y_f])
    d = np.linalg.norm(P - P_f)
    global zeta
    f = 0.0
    if(d<=d_0):
        f = -zeta*(P - P_f)
    else:
        f = -d_0*zeta*(P - P_f)/d
    return f

def f_rep(x, y, x_o, y_o, rho_0):
    P = np.array([x, y])
    P_o = np.array([x_o, y_o])
    rho = np.linalg.norm(P - P_o)
    global eta
    f = 0.0
    if(rho<=rho_0):
        f = eta * ((1/rho) - (1/rho_0)) * ((P - P_o) / (rho**1.5))
    else:
        f = 0
    return f

def f_total(x, y, x_f, y_f, x_o, y_o, d_0, rho_0):
    f_a = f_att(x, y, x_f, y_f, d_0)
    f_r = f_rep(x, y, x_o, y_o, rho_0)
    return f_a + f_r

def get_new_xy(q,F,alpha):
    return q + alpha*F

# f_net = f_total(x, y, x_f, y_f, x_o, y_o, d_0, rho_0)

# def disp_vector_field(ox,oy,vx,vy):
#     plt.quiver(ox,oy,vx,vy, units='xy', linewidth=0.2, color='b')

# x_arr = np.linspace(-2,2,15)
# y_arr = np.linspace(-2,2,15)

# plt.plot(x_f,y_f,'ro')
# plt.plot(x_o,y_o,'ko')

# for x in x_arr:
#     for y in y_arr:
#         f_net = f_total(x, y, x_f, y_f, x_o, y_o, d_0, rho_0)
#         disp_vector_field(x,y,f_net[0],f_net[1])

# plt.title('Vector field')
# plt.xlim(-3, 3)
# plt.ylim(-3, 3)
# plt.grid()
# plt.show()
    
