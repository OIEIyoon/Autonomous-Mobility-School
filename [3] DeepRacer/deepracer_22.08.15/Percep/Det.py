import numpy as np
import cv2
import math

from Percep.utils.func import Homography, LaneDet



def max_curv(right_curv, left_curv, max_K_pre, alpha_curv):
    if right_curv is None and left_curv is None:
        max_K = max_K_pre
    else:
        if right_curv is not None and left_curv is not None:
            max_K = right_curv[0] + left_curv[0]
        elif right_curv is None and left_curv is not None:
            max_K = left_curv[0] * 2
        elif right_curv is not None and left_curv is None:
            max_K = right_curv[0] * 2
        max_K = max_K * alpha_curv + (1 - alpha_curv) * max_K_pre

    max_K = min(3, max(0, abs(max_K)))

    return max_K


class LeftLane:
    def __init__(self, H, w = 0.5, lower = (0, 200, 180), upper = (100, 255, 255)):
        self.w = w
        self.lower = lower
        self.upper = upper

        self.left_line = [0, w / 2]
        self.left_curv = [0, 0, w / 2]

        ## pre lane
        self.left_line_pre = self.left_line
        self.left_curv_pre = self.left_curv

        self.H = H

    def backup(self):
        self.left_line_pre = self.left_line
        self.left_curv_pre = self.left_curv


    def get_lane(self, frame,
                 crop_lineX_lower = 0.2, crop_lineX_upper = 0.5, crop_curvX_lower = 0.4, crop_curvX_upper = 0.7, y1 = 0.3, y2 = 0.4):
        try:
            left_lane, line, curv = LaneDet(frame, self.H, self.lower, self.upper,
                 crop_lineX_lower, crop_lineX_upper, crop_curvX_lower, crop_curvX_upper, y1, y2)
            self.left_line = left_lane[0]
        except Exception as ex:
            self.left_line = None
            pass
        try:
            self.left_curv = left_lane[1]
        except:
            self.left_curv = None
            pass


class RightLane:
    def __init__(self, H, w = 0.5, lower = (235, 100, 110), upper = (255, 180, 255)):
        self.w = w
        self.lower = lower
        self.upper = upper

        self.right_line = [0, -w / 2]
        self.right_curv = [0, 0, -w / 2]

        ## pre lane
        self.right_line_pre = self.right_line
        self.right_curv_pre = self.right_curv


        self.H = H


    def backup(self):
        self.right_line_pre = self.right_line
        self.right_curv_pre = self.right_curv


    def get_lane(self, frame,
                 crop_lineX_lower, crop_lineX_upper, crop_curvX_lower, crop_curvX_upper, y1 = 0.3, y2 = 0.4):
        try:
            right_lane, line, curv = LaneDet(frame, self.H, self.lower, self.upper,
                                 crop_lineX_lower, crop_lineX_upper, crop_curvX_lower, crop_curvX_upper, y1, y2)
            self.right_line = right_lane[0]
        except Exception as ex:
            #print(ex)
            right_line = None
            pass
        try:
            self.right_curv = right_lane[1]
        except:
            self.right_curv = None
            pass


class Lane:
    def __init__(self, H, w = 0.5, left_lower = (0, 200, 180), left_upper = (100, 255, 255),
                 right_lower = (235, 100, 110), right_upper = (255, 180, 255)):
        self.H = H

        # lane
        self.right_lane = RightLane(self.H, w, right_lower, right_upper)
        self.left_lane = LeftLane(self.H, w, left_lower, left_upper)

        self.max_K = 0
        self.max_K_pre = 0

        self.detected_lane_counts = 0


    def get_lane(self, frame, alpha_curv= 0.1,
                 crop_lineX_lower=0.2, crop_lineX_upper=0.5, crop_curvX_lower=0.4, crop_curvX_upper=0.7, y1 = 0.3, y2 = 0.4):
        self.detected_lane_counts = 0

        self.right_lane.get_lane(frame,
                                 crop_lineX_lower, crop_lineX_upper, crop_curvX_lower, crop_curvX_upper, y1, y2)
        self.left_lane.get_lane(frame,
                                 crop_lineX_lower, crop_lineX_upper, crop_curvX_lower, crop_curvX_upper, y1, y2)

        self.max_K = max_curv(self.right_lane.right_curv, self.left_lane.left_curv, self.max_K_pre, alpha_curv)

        # detection check
        if self.left_lane.left_line is not None:
            self.detected_lane_counts = self.detected_lane_counts + 1
        if self.right_lane.right_line is not None:
            self.detected_lane_counts = self.detected_lane_counts + 2


    def backup(self):
        self.right_lane.backup()
        self.left_lane.backup()
        self.max_K_pre = self.max_K


class StopLine:
    def __init__(self, H, pre_clearance, alpha_c = 0.9, lower_rgb = (0, 140, 230), upper_rgb = (100, 200, 255)):
        self.H = H

        self.isTarget = 0
        self.clearance = None

        self.lower_rgb = lower_rgb
        self.upper_rgb = upper_rgb

        self.pre_clearance = pre_clearance
        self.alpha_c = alpha_c


    def StopLineDet(self, frame,
                    crop_lineX_lower=0, crop_lineX_upper=1, y1 = 0.3):
        lane, line, curv = LaneDet(frame, self.H, lower_rgb = self.lower_rgb, upper_rgb = self.upper_rgb,
            crop_lineX_lower = crop_lineX_lower, crop_lineX_upper = crop_lineX_upper, y1 = y1, y2 = 0)
        real_x = line[0]
        real_y = line[1]

        try:
            poly_coef_1st = np.polyfit(real_x, real_y, 1)

            self.isTarget = 1
            a = poly_coef_1st[0]
            b = poly_coef_1st[1]
            self.clearance = min(abs(b / math.sqrt(1 + a ** 2)), self.pre_clearance)

            return self.isTarget, self.clearance
        except Exception as ex:
            poly_coef_1st = None
            pass


    def backup(self):
        if self.clearance is not None:
            self.pre_clearance = self.clearance * (1 - self.alpha_c) + self.pre_clearance * self.alpha_c



if __name__ == "__main__":
    pointx = [62, 124, 466, 542]
    pointy = [346, 277, 272, 349]
    realx = [0.31, 0.455, 0.455, 0.30]
    realy = [0.15, 0.15, -0.14, -0.14]
    H = Homography(pointx, pointy, realx, realy)

    lane = Lane(H, w = 0.5, left_lower = (100, 225, 100), left_upper = (105, 235, 105),
                right_lower = (225, 100, 100), right_upper = (235, 105, 105))
    stopline = StopLine(H, 2, 0.9, (100, 100, 225), (105, 105, 235))

    frame = cv2.imread('capture_f0.png', cv2.IMREAD_COLOR)

    lane.get_lane(frame, alpha_curv=0.1,
                  crop_lineX_lower=0.2, crop_lineX_upper=0.4, crop_curvX_lower=0.3, crop_curvX_upper=0.6, y1=0.3, y2=0.4)
    if lane.detected_lane_counts == 0:
        print("BLIND")
    elif lane.detected_lane_counts == 1:
        print("Left Lane")
    elif lane.detected_lane_counts == 2:
        print("Right Lane")
    else:
        print("Both Lanes")

    # stop line detection
    stopline.StopLineDet(frame,
                         crop_lineX_lower=0, crop_lineX_upper=1, y1=0.3)
    if stopline.clearance is not None:
        print("##### STOP #####")
    else:
        print("Go")
