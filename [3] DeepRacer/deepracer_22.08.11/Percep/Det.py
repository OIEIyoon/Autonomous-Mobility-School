import numpy as np
import cv2
import math

from Percep.utils.func import Homography

def LaneDet(frame, H, lower_rgb, upper_rgb,
            crop_lineX_lower = 0.3, crop_lineX_upper = 0.6, crop_curvX_lower = 0.5, crop_curvX_upper = 1):
    img_color = frame
    img_rgb = cv2.cvtColor(img_color, cv2.COLOR_BGR2RGB)
    img_mask = cv2.inRange(img_rgb, lower_rgb, upper_rgb)
    img_mask = (np.array(img_mask))
    coord = np.where(img_mask >= 1)
    coord = np.array([coord[1], coord[0]])
    ones_vec = np.ones((1, np.size(coord[0])))[0]
    coord = np.insert(coord, 2, [ones_vec], axis=0)

    homo = np.dot(H, coord)
    lane_points = [homo[0] / homo[2], homo[1] / homo[2]]


    ################
    ##### CROP #####
    ################
    idx_x1, idx_x2 = np.where(lane_points[0] > crop_lineX_lower), np.where(lane_points[0] < crop_lineX_upper)
    idx_y1, idx_y2 = np.where(lane_points[1] > -0.3), np.where(lane_points[1] < 0.3)
    idx_x = np.intersect1d(idx_x1, idx_x2)
    idx_y = np.intersect1d(idx_y1, idx_y2)
    idx = np.intersect1d(idx_x, idx_y)
    real_x = lane_points[0][idx]
    real_y = lane_points[1][idx]

    idx_x1_curv, idx_x2_curv = np.where(lane_points[0] > crop_curvX_lower), np.where(lane_points[0] < crop_curvX_upper)
    idx_y1_curv, idx_y2_curv = np.where(lane_points[1] > -0.4), np.where(lane_points[1] < 0.4)
    idx_x_curv = np.intersect1d(idx_x1_curv, idx_x2_curv)
    idx_y_curv = np.intersect1d(idx_y1_curv, idx_y2_curv)
    idx_curv = np.intersect1d(idx_x_curv, idx_y_curv)
    real_x_curv = lane_points[0][idx_curv]
    real_y_curv = lane_points[1][idx_curv]

    # xq1 = np.linspace(crop_lineX_lower-0.5,crop_lineX_upper+0.5,50)
    # xq2 = np.linspace(crop_curvX_lower-0.5,crop_curvX_upper+0.5,50)
    try:
        # poly_coeff_1st = RANSAC(real_x, real_y, 1)
        poly_coeff_1st = np.polyfit(real_x, real_y, 1)
        # yq1 = np.polyval(poly_coeff_1st, xq1)
        # plt.scatter(real_x, real_y, s = 5)
        # plt.plot(xq1,yq1, 'k-', linewidth = 2)
    except Exception as ex:
        poly_coeff_1st = None
        # print("error")
        # print(ex)
        # print(traceback.print_exc())
        # yq1 = None
        pass

    try:
        # poly_coeff_2nd = RANSAC(real_x_curv, real_y_curv, 2)
        poly_coeff_2nd = np.polyfit(real_x_curv, real_y_curv, 2)
        # print("test: ", poly_coeff_2nd)
        # yq2 = np.polyval(poly_coeff_2nd, xq2)
        # poly_coeff_2nd2 = np.polyfit(real_x_curv, real_y_curv, 2)
        # yq22 = np.polyval(poly_coeff_2nd2, xq2)
        # plt.scatter(real_x_curv, real_y_curv, s = .2)
        # plt.plot(xq2,yq2, 'k:')
        # plt.plot(xq2,yq22, 'm-', linewidth = 2)
    except Exception as ex:
        poly_coeff_2nd = None
        # print("error")
        # print(ex)
        # print(traceback.print_exc())
        # yq2 = None
        pass

    return [poly_coeff_1st, poly_coeff_2nd]

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
                 crop_lineX_lower = 0.3, crop_lineX_upper = 0.6, crop_curvX_lower = 0.5, crop_curvX_upper = 1):
        try:
            left_lane = LaneDet(frame, self.H, self.lower, self.upper,
                 crop_lineX_lower, crop_lineX_upper, crop_curvX_lower, crop_curvX_upper)
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
                 crop_lineX_lower, crop_lineX_upper, crop_curvX_lower, crop_curvX_upper):
        try:
            right_lane = LaneDet(frame, self.H, self.lower, self.upper,
                                 crop_lineX_lower, crop_lineX_upper, crop_curvX_lower, crop_curvX_upper)
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
                 crop_lineX_lower=0.3, crop_lineX_upper=0.6, crop_curvX_lower=0.5, crop_curvX_upper=1):
        self.detected_lane_counts = 0

        self.right_lane.get_lane(frame,
                                 crop_lineX_lower, crop_lineX_upper, crop_curvX_lower, crop_curvX_upper)
        self.left_lane.get_lane(frame,
                                 crop_lineX_lower, crop_lineX_upper, crop_curvX_lower, crop_curvX_upper)

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
        self.alpha_c = 0.9


    def StopLineDet(self, frame,
                    crop_lineX_lower=0, crop_lineX_upper=1):
        img_color = frame

        img_rgb = cv2.cvtColor(img_color, cv2.COLOR_BGR2RGB)
        img_mask = cv2.inRange(img_rgb, self.lower_rgb, self.upper_rgb)
        img_mask = (np.array(img_mask))
        coord = np.where(img_mask >= 1)
        coord = np.array([coord[1], coord[0]])
        ones_vec = np.ones((1, np.size(coord[0])))[0]
        coord = np.insert(coord, 2, [ones_vec], axis=0)

        homo = np.dot(self.H, coord)
        lane_points = [homo[0] / homo[2], homo[1] / homo[2]]


        ################
        ##### CROP #####
        ################
        idx_x1, idx_x2 = np.where(lane_points[0] > crop_lineX_lower), np.where(lane_points[0] < crop_lineX_upper)
        idx_y1, idx_y2 = np.where(lane_points[1] > -0.3), np.where(lane_points[1] < 0.3)
        idx_x = np.intersect1d(idx_x1, idx_x2)
        idx_y = np.intersect1d(idx_y1, idx_y2)
        idx = np.intersect1d(idx_x, idx_y)
        real_x = lane_points[0][idx]
        real_y = lane_points[1][idx]

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