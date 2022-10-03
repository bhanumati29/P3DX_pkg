import math
import numpy as np
import matplotlib.pyplot as plt

r = 0.125/2.0
L = 0.25

w_max = 250.0*(2*math.pi/60.0)
w_min = -250.0*(2*math.pi/60.0)

w_arr = np.linspace(w_min,w_max,100)

Vc_arr = []
Wc_arr = []

i = 1
for w1 in w_arr:
    for w2 in w_arr:
        print(i, w1,w2)
        i += 1
        Vc = (w1+w2)*r/2.0
        Wc = (w1-w2)*r/L
        Vc_arr.append(Vc)
        Wc_arr.append(Wc*180.0/math.pi)

plt.title('DIfferential drive feasible velocity region')
plt.xlabel('Vc (m/sec)')
plt.ylabel('Wc, (rad/sec)')
plt.grid()
plt.plot(Vc_arr, Wc_arr,'ko')
plt.show()