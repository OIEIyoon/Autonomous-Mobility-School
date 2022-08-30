import numpy as np
import numpy.linalg as lin
import matplotlib.pyplot as plt
import cv2



def Crop(lane_points, crop_lineX_lower, crop_lineX_upper, y):
    idx_x1, idx_x2 = np.where(lane_points[0] > crop_lineX_lower), np.where(lane_points[0] < crop_lineX_upper)
    idx_y1, idx_y2 = np.where(lane_points[1] > -1 * y), np.where(lane_points[1] < y)
    idx_x = np.intersect1d(idx_x1, idx_x2)
    idx_y = np.intersect1d(idx_y1, idx_y2)
    idx = np.intersect1d(idx_x, idx_y)
    real_x = lane_points[0][idx]
    real_y = lane_points[1][idx]

    return real_x, real_y



def Homography(*args):
    pointx = args[0]
    pointy = args[1]
    realx = args[2]
    realy = args[3]

    pts_src = np.array(
        [[pointx[0], pointy[0]], [pointx[1], pointy[1]], [pointx[2], pointy[2]], [pointx[3], pointy[3]]])
    pts_dst = np.array([[realx[0], realy[0]], [realx[1], realy[1]], [realx[2], realy[2]], [realx[3], realy[3]]])
    # Calculate Homography
    h, status = cv2.findHomography(pts_src, pts_dst)

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



def display_lines(frame, lines, line_color=(0, 255, 0), line_width=2):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image



def get_points(frame, H, lower_rgb, upper_rgb):
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

    return lane_points



def LaneDet(frame, H, lower_rgb, upper_rgb,
            crop_lineX_lower = 0.3, crop_lineX_upper = 0.6, crop_curvX_lower = 0.5, crop_curvX_upper = 1, y1 = 0.3, y2 = 0.4):
    lane_points = get_points(frame, H, lower_rgb, upper_rgb)

    real_x, real_y = Crop(lane_points, crop_lineX_lower, crop_lineX_upper, y1)
    real_x_curv, real_y_curv = Crop(lane_points, crop_curvX_lower, crop_curvX_upper, y2)

    try:
        poly_coeff_1st = np.polyfit(real_x, real_y, 1)
    except Exception as ex:
        poly_coeff_1st = None
        pass

    try:
        poly_coeff_2nd = np.polyfit(real_x_curv, real_y_curv, 2)
    except Exception as ex:
        poly_coeff_2nd = None
        pass

    return [poly_coeff_1st, poly_coeff_2nd], [real_x, real_y], [real_x_curv, real_y_curv]
    
