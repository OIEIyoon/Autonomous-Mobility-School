import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d


lower_red = np.array([245, 100, 110], dtype="uint8")
upper_red = np.array([255, 180, 255], dtype="uint8")
lower_green = np.array([0, 200, 180], dtype="uint8")
upper_green = np.array([100, 255, 255], dtype="uint8")
lower_white = np.array([240, 240, 240], dtype="uint8")
upper_white = np.array([255, 255, 255], dtype="uint8")
lower_blue = np.array([0, 140, 220], dtype="uint8")
upper_blue = np.array([100, 200, 255], dtype="uint8")


class Car:
    def __init__(self):
        self.u = 0
        self.Vx = 0
        self.Ax = 0
        ## pre state
        self.u_pre = 0
        self.Vx_pre = 0
        self.Ax_pre = 0


    def backup(self):
        self.u_pre = self.u
        self.Vx_pre = self.Vx
        self.Ax_pre = self.Ax_pre


class Info:
    def __init__(self, size = 100000):
        self.l = 0
        self.size = size

        self.times = [0 for i in range(size)]
        self.e_ys = [0 for i in range(size)]
        self.e_as = [0 for i in range(size)]
        self.Axs = [0 for i in range(size)]
        self.Ays = [0 for i in range(size)]
        self.deltas = [0 for i in range(size)]
        self.Vxs = [0 for i in range(size)]
        self.Vxs_des = [0 for i in range(size)]
        self.dls = [0 for i in range(size)]

        self.r_cnt = 0

    def get_info(self, *args):
        self.times[self.l] = args[0]
        self.e_ys[self.l] = args[1]
        self.e_as[self.l] = args[2]
        self.Axs[self.l] = args[3]
        self.Ays[self.l] = args[4]
        self.deltas[self.l] = args[5]
        self.Vxs[self.l] = args[6]
        self.Vxs_des[self.l] = args[7]
        self.dls[self.l] = args[8]

        self.l = (self.l + 1) % self.size


    def save(self, path):
        times = self.times[0:self.l]
        e_ys = self.e_ys[0:self.l]
        e_as = self.e_as[0:self.l]
        Axs = self.Axs[0:self.l]
        Ays = self.Ays[0:self.l]
        deltas = self.deltas[0:self.l]
        Vxs = self.Vxs[0:self.l]
        Vxs_des = self.Vxs_des[0:self.l]
        #dls = self.dls[0:self.l]

        # smooth
        e_ys = interp1d(times, e_ys, kind='cubic')
        e_as = interp1d(times, e_as, kind='cubic')
        Axs = interp1d(times, Axs, kind='cubic')
        Ays = interp1d(times, Ays, kind='cubic')
        deltas = interp1d(times, deltas, kind='cubic')
        Vxs = interp1d(times, Vxs, kind='cubic')
        Vxs_des = interp1d(times, Vxs_des, kind='cubic')

        # draw plot
        plt.subplots_adjust(left=0.125, bottom=0.1, right=0.9, top=0.9, wspace=0.4, hspace=0.5)
        plt.grid(True)

        plt.subplot(3, 2, 1)
        plt.plot(times, e_ys)
        plt.ylim(-0.5, 0.5)
        plt.xlabel('time[s]')
        plt.ylabel('e_y[m]')

        plt.subplot(3, 2, 2)
        plt.plot(times, e_as)
        plt.ylim(-50, 50)
        plt.xlabel('time[s]')
        plt.ylabel('e_a[deg]')

        plt.subplot(3, 2, 3)
        plt.plot(times, Axs, label='longitudinal', color='b')
        plt.plot(times, Ays, label='lateral', color='r')
        plt.ylim(-3, 3)
        plt.xlabel('time[s]')
        plt.ylabel('acc')
        plt.legend(loc='upper right', prop={'size': 6})

        plt.subplot(3, 2, 4)
        plt.plot(times, deltas)
        plt.ylim(-50, 50)
        plt.xlabel('time[s]')
        plt.ylabel('delta[deg]')

        plt.subplot(3, 2, 5)
        plt.plot(times, Vxs, label='act', linestyle='solid')
        plt.plot(times, Vxs_des, label='des', linestyle='dashed')
        plt.ylim(0, 3)
        plt.xlabel('time[s]')
        plt.ylabel('Vx')
        plt.legend(loc='upper right', prop={'size': 6})

        """
        # 0: empty 1: left detected 2: right: detected 3: two lane detected
        plt.subplot(3, 2, 6)
        plt.plot(times, dls)
        plt.ylabel("detected_lane")
        """

        # save plot
        print("result_plot" + str(self.r_cnt) + "save")
        plt.savefig(path + "/result" + str(self.r_cnt) + ".png")
        plt.close()

        self.r_cnt += 1
