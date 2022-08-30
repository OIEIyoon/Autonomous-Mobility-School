import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
import numpy.linalg as lin

import Percep.Det as Det


def get_camera():
    VIDEO = "/dev/video"
    cap = None
    v_num = 0

    for i in range(10):
        try:
            cap = cv.VideoCapture("/dev/video" + str(v_num))
            ret, frame = cap.read()

            if ret == True:
                break
        except:
            pass

    return cap



def Homography():
    pointx = [62, 124, 466, 542]
    pointy = [346, 277, 272, 349]
    realx = [0.31, 0.455, 0.455, 0.30]
    realy = [0.15, 0.15, -0.14, -0.14]

    pts_src = np.array([[pointx[0], pointy[0]], [pointx[1], pointy[1]], [pointx[2], pointy[2]], [pointx[3], pointy[3]]])
    pts_dst = np.array([[realx[0], realy[0]], [realx[1], realy[1]], [realx[2], realy[2]], [realx[3], realy[3]]])
    # Calculate Homography
    h, status = cv.findHomography(pts_src, pts_dst)
    return h



def RANSAC(x, y, n):
    N = 30
    T = 0.02
    n_sample = 100
    max_cnt = 0
    best_model = [0, 0, 0]
    curv_param = 1.0
    n_data = len(x)
    x = np.array([x]).T
    y = np.array([y]).T

    if n == 2:
        A = np.hstack([np.square(x), x, np.ones((n_data, 1))])
    else:
        A = np.hstack([x, np.ones((n_data, 1))])
    B = y

    for itr in range(N):
        k = np.floor(n_data * np.random.rand(n_sample))
        k = k.astype(int)
        if n == 2:
            AA = np.hstack([np.square(x[k]), x[k], np.ones((len(k), 1))])
        else:
            AA = np.hstack([x[k], np.ones((len(k), 1))])
        BB = y[k]
        X = np.dot(lin.pinv(AA), BB)
        residual = abs(B - np.dot(A, X))
        cnt = len([idx for idx, val in enumerate(residual) if val < T])
        if cnt > max_cnt:
            best_model = X
            max_cnt = cnt

    residual = abs(np.dot(A, best_model) - B)
    in_k = [idx for idx, val in enumerate(residual) if val < T]

    if n == 2:
        A2 = np.hstack([np.square(x[in_k]), x[in_k], np.ones((len(in_k), 1))])
    else:
        A2 = np.hstack([x[in_k], np.ones((len(in_k), 1))])
    B2 = y[in_k]
    X = np.dot(lin.pinv(A2), B2)
    X[0] = X[0] / curv_param
    return X



def LaneDet(img_rgb, lower, upper, crop_lineX_lower, crop_lineX_upper, crop_curvX_lower, crop_curvX_upper, rgb):
    img_mask = cv.inRange(img_rgb, lower, upper)
    img_mask = (np.array(img_mask))
    coord = np.where(img_mask >= 1)
    coord = np.array([coord[1], coord[0]])
    ones_vec = np.ones((1, np.size(coord[0])))[0]
    coord = np.insert(coord, 2, [ones_vec], axis=0)

    H = Homography()

    homo = np.dot(H, coord)
    lane_points = [homo[0] / homo[2], homo[1] / homo[2]]

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

    xq1 = np.linspace(crop_lineX_lower - 0.5, crop_lineX_upper + 0.5, 50)
    xq2 = np.linspace(crop_curvX_lower - 0.5, crop_curvX_upper + 0.5, 50)
    try:
        # poly_coeff_1st = RANSAC(real_x, real_y, 1)
        poly_coeff_1st = np.polyfit(real_x, real_y, 1)
        yq1 = np.polyval(poly_coeff_1st, xq1)
        plt.scatter(real_x, real_y, color=rgb, s=5)
        plt.plot(xq1, yq1, 'k-', linewidth=2)
    except Exception as ex:
        # print("error")
        # print(ex)
        # print(traceback.print_exc())
        poly_coeff_1st = None
        yq1 = None
        pass

    try:
        # poly_coeff_2nd = RANSAC(real_x_curv, real_y_curv, 2)
        poly_coeff_2nd = np.polyfit(real_x_curv, real_y_curv, 2)
        yq2 = np.polyval(poly_coeff_2nd, xq2)
        poly_coeff_2nd2 = np.polyfit(real_x_curv, real_y_curv, 2)
        yq22 = np.polyval(poly_coeff_2nd2, xq2)
        plt.scatter(real_x_curv, real_y_curv, color=[.7 * _ for _ in rgb], s=.2)
        plt.plot(xq2, yq2, 'k:')
        plt.plot(xq2, yq22, 'm-', linewidth=2)
    except Exception as ex:
        # print("error")
        # print(ex)
        # print(traceback.print_exc())
        poly_coeff_2nd = None
        yq2 = None
        pass

    print(poly_coeff_1st)
    print(poly_coeff_2nd)
    return img_mask



def capture(path, frame, cnt):
    crop_lineX_lower = 0.3
    crop_lineX_upper = 0.6
    crop_curvX_lower = 0.5
    crop_curvX_upper = 1
    img_color = frame
    height, width, _ = img_color.shape
    img_rgb = cv.cvtColor(img_color, cv.COLOR_BGR2RGB)
    lower_red = np.array([245, 100, 110], dtype="uint8")
    upper_red = np.array([255, 180, 255], dtype="uint8")
    lower_green = np.array([0, 200, 180], dtype="uint8")
    upper_green = np.array([100, 255, 255], dtype="uint8")
    lower_white = np.array([240, 240, 240], dtype="uint8")
    upper_white = np.array([255, 255, 255], dtype="uint8")
    lower_blue = np.array([0, 140, 220], dtype="uint8")
    upper_blue = np.array([100, 200, 255], dtype="uint8")

    plt.grid(True)
    plt.axis("equal")
    img_mask_L = LaneDet(img_rgb, lower_red, upper_red, crop_lineX_lower, crop_lineX_upper, crop_curvX_lower,
                             crop_curvX_upper, [1, 0, 0])
    img_mask_R = LaneDet(img_rgb, lower_green, upper_green, crop_lineX_lower, crop_lineX_upper, crop_curvX_lower,
                     crop_curvX_upper, [0, 1, 0])
    img_mask_S = LaneDet(img_rgb, lower_blue, upper_blue, 0, 1, 0, 0, [0, 0, 1])

    plt.xlim(0, 1.3)
    plt.ylim(-0.5, 0.5)
    # plt.show()

    print("Capture!")
    plt.savefig(path + "capture" + str(cnt) + ".png")
    plt.close()
