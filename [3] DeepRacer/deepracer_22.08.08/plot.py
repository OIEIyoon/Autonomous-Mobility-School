import matplotlib.pyplot as plt
import math
import os
import time
import uuid

def save_log(name, data):
    with open('./log/' + str(name) + '.txt', 'w') as f:
        for line in data:
            f.write("%s\n" %line)


def log_plot(l, times, e_ys, e_as, Axs, Ays, deltas, Vxs, Vxs_des, dls, cnt):
    h = 6
    w = 6
    
    times = times[0:l]
    e_ys = e_ys[0:l]
    e_as = e_as[0:l]
    Axs = Axs[0:l]
    Ays = Ays[0:l]
    deltas = deltas[0:l]
    Vxs = Vxs[0:l]
    Vxs_des = Vxs_des[0:l]
    dls = dls[0:l]
    
    plt.subplots_adjust(left = 0.125, bottom = 0.1, right = 0.9, top = 0.9, wspace = 0.4, hspace = 0.5)
    plt.grid(True)
    
    plt.subplot(3,2,1)
    #plt.figure(figsize = (h,w))
    plt.plot(times, e_ys)
    plt.ylim(-0.5, 0.5)
    plt.xlabel('time[s]')
    plt.ylabel('e_y[m]')

    plt.subplot(3,2,2)
    #plt.figure(figsize = (h,w))
    plt.plot(times, e_as)
    plt.ylim(-50, 50)
    plt.xlabel('time[s]')
    plt.ylabel('e_a[deg]')

    plt.subplot(3,2,3)
    #plt.figure(figsize = (h,w))
    plt.plot(times, Axs, label = 'longitudinal', color = 'b')
    plt.plot(times, Ays, label = 'lateral', color = 'r')
    plt.ylim(-3, 3)
    plt.xlabel('time[s]')
    plt.ylabel('acc')
    plt.legend(loc='upper right', prop = {'size': 6})


    plt.subplot(3,2,4)
    #plt.figure(figsize = (h,w))
    plt.plot(times, deltas)
    plt.ylim(-50, 50)
    plt.xlabel('time[s]')
    plt.ylabel('delta[deg]')

    plt.subplot(3,2,5)
    #plt.figure(figsize = (h,w))
    plt.plot(times, Vxs, label='act', linestyle='solid')
    plt.plot(times, Vxs_des, label='des', linestyle='dashed')
    plt.ylim(0, 3)
    plt.xlabel('time[s]')
    plt.ylabel('Vx')
    plt.legend(loc='upper right', prop = {'size': 6})
    
    # 0: empty 1: left detected 2: right: detected 3: two lane detected
    plt.subplot(3,2,6)
    #plt.figure(figsize = (h,w))
    plt.plot(times, dls)
    plt.ylabel("detected_lane")
    
    
    #plt.show()
    print("result_plot" + str(cnt) + "save")
    plt.savefig(f"result" + str(cnt) + ".png")
    plt.close()
