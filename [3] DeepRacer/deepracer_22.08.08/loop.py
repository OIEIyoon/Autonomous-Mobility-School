import error
import controller
import plot

import matplotlib.pyplot as plt
import math
import time
import os

import pandas as pd

LEFT = "left1.xlsx"
RIGHT = "right1.xlsx"

left = pd.read_excel("./data/" + LEFT, engine='openpyxl')
right = pd.read_excel("./data/" + RIGHT, engine='openpyxl')

## pre lane
points = 21
pre_coef = [0, 0, 0.2]
pre_left_lane_pos = [[0 for i in range(points)] for j in range(2)]
pre_right_lane_pos = [[0 for i in range(points)] for j in range(2)]

## pre curvature
pre_curv = 0

## state info
Vx = 0
dt = 0.1
Vx_pre = 0
Ax_pre = 0

## update rate
alpha = 0.3

## lane width
w = 0.6

## log info
l = 0
size = 100
times = [0 for i in range(size)]
e_ys = [0 for i in range(size)]
e_as = [0 for i in range(size)]
Axs = [0 for i in range(size)]
Ays = [0 for i in range(size)]
deltas = [0 for i in range(size)]
Vxs = [0 for i in range(size)]
Vxs_des = [0 for i in range(size)]

######
#test#
######
for k in range(0, len(left.index), 2):
    left_pos = []
    right_pos = []

    left_pos.append(list(left.iloc[k]))
    left_pos.append(list(left.iloc[k + 1]))
    right_pos.append(list(right.iloc[k]))
    right_pos.append(list(right.iloc[k + 1]))

    ## get error
    coef = error.route(pre_coef, pre_left_lane_pos, pre_right_lane_pos, left_pos, right_pos, alpha = alpha, w = w)
    A, B, e_y, e_a = error.get_error(coef)
    #print("A: ", A , " B: ", B)
    prev_k, traj_max_k = error.curvature(coef, A, B, 1)

    ## curv filtering
    curv_road = traj_max_k * 0.3 + 0.7 * pre_curv

    #print("prev_curvature: ", prev_k, " traj_curvature: ", curv_road, " e_y: ", e_y, " e_a: ", e_a * 180 / math.pi)

    ############
    #controller#
    ############
    u = controller.Lateral_control(e_y, e_a)
    #print("steer_angle: ", u)

    Ax, Vx_des, cl_des = controller.Longitudinal_control(Ax_pre = Ax_pre, Vx = Vx, dt = dt, curv_road = curv_road, isTarget = 0, clearance = None)
    Vx = Vx_pre + Ax * dt  # option
    #print("Ax: ", Ax, "Vx: ", Vx, "Vx_des: ", Vx_des, "cl_des: ", cl_des ,'\n')

    t = time.time() # one loop time

    ## save info
    ## Vx is gotten from pwm module
    times[l] = t
    e_ys[l] = e_y
    e_as[l] = e_a * 180.0 / math.pi
    Axs[l] = Ax
    Ays[l] = -Vx**2 / 0.16 * math.tan(u)
    deltas[l] = u * 180.0 / math.pi
    Vxs[l] = Vx
    Vxs_des[l] = Vx_des
    l = (l+1) % 100

    ## update
    pre_curv = curv_road
    pre_coef = coef
    Vx_pre = Vx
    Ax_pre = Ax



# plot
plot.log_plot(l, times, e_ys, e_as, Axs, Ays, deltas, Vxs, Vxs_des)


# save log
if os.path.isdir("./log"):
    plot.save_log('times', times)
    plot.save_log('e_ys', e_ys)
    plot.save_log('e_as', e_as)
    plot.save_log('Axs', Axs)
    plot.save_log('Ays', Ays)
    plot.save_log('deltas', deltas)
    plot.save_log('Vxs', Vxs)
    plot.save_log('Vxs_des', Vxs_des)
else:
    os.makedirs("./log")