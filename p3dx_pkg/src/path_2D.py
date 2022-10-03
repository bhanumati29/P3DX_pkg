def get_st_line(x1=0,y1=0,x2=0,y2=0):
    x_traj = [-1.6]
    y_traj = [0]
    iii = 0
    while(x_traj[iii]<=1.8):
        x_n = x_traj[iii] + 0.1
        y_n = 0.0
        x_traj.append(x_n)
        y_traj.append(y_n)
        iii += 1
    return x_traj, y_traj

x_traj, y_traj = get_st_line()
print(x_traj, y_traj)