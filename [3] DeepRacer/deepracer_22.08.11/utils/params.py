import cv2
import matplotlib.pyplot as plt
import numpy as np
import os

from scipy.interpolate import interp1d
from scipy.interpolate import make_interp_spline, BSpline

from Percep.utils.func import Homography


# CROP
crop_lineX_lower = 0.3
crop_lineX_upper = 0.6
crop_curvX_lower = 0.5
crop_curvX_upper = 1


# RGB
lower_red = np.array([245, 100, 110], dtype="uint8")
upper_red = np.array([255, 180, 255], dtype="uint8")
lower_green = np.array([0, 200, 180], dtype="uint8")
upper_green = np.array([100, 255, 255], dtype="uint8")
lower_white = np.array([240, 240, 240], dtype="uint8")
upper_white = np.array([255, 255, 255], dtype="uint8")
lower_blue = np.array([0, 140, 220], dtype="uint8")
upper_blue = np.array([100, 200, 255], dtype="uint8")


# Homographic view matrix
pointx = [62, 124, 466, 542]
pointy = [346, 277, 272, 349]
realx = [0.31, 0.455, 0.455, 0.30]
realy = [0.15, 0.15, -0.14, -0.14]
H = Homography(pointx, pointy, realx, realy)


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


    def save(self, path, K):
        times = self.times[0:self.l]
        e_ys = self.e_ys[0:self.l]
        e_as = self.e_as[0:self.l]
        Axs = self.Axs[0:self.l]
        Ays = self.Ays[0:self.l]
        deltas = self.deltas[0:self.l]
        Vxs = self.Vxs[0:self.l]
        Vxs_des = self.Vxs_des[0:self.l]
        dls = self.dls[0:self.l]

        # smooth
        times_new = np.linspace(times[0], times[-1], int(self.l * 5/3))
        spl = make_interp_spline(times, e_ys, k=3)  # type: BSpline
        e_ys = spl(times_new)
        spl = make_interp_spline(times, e_as, k=3)  # type: BSpline
        e_as = spl(times_new)
        spl = make_interp_spline(times, Axs, k=3)  # type: BSpline
        Axs = spl(times_new)
        spl = make_interp_spline(times, Ays, k=3)  # type: BSpline
        Ays = spl(times_new)
        spl = make_interp_spline(times, deltas, k=3)  # type: BSpline
        deltas = spl(times_new)
        spl = make_interp_spline(times, Vxs, k=3)  # type: BSpline
        Vxs = spl(times_new)
        spl = make_interp_spline(times, Vxs_des, k=3)  # type: BSpline
        Vxs_des = spl(times_new)
        times = times_new


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
        if not os.path.isdir(path):
            print("No directory, create directory")
            os.makedirs(path)

        name = str(K[0]) + '_' + str(K[1]) + '_' + str(K[2]) + '_' + str(K[3]) + '_' + str(K[4]) + '_' + str(K[5]) + '_' + str(K[6])
        plt.savefig(path + '/' + name + '.png')
        plt.close()

        self.r_cnt += 1
