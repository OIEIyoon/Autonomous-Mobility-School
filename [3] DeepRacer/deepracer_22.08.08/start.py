from pwm import PWM
import cv2
import numpy as np
import matplotlib.pyplot as plt
import numpy.linalg as lin

import error
import controller
import plot
import math
import time
import motor
import traceback
import stop
from keyPoller import KeyPoller

import LaneDet_Test

cap = cv2.VideoCapture("/dev/video1")


def Homography():
    pointx = [62, 124, 466, 542]
    pointy = [346, 277, 272, 349]
    realx = [0.31, 0.455, 0.455, 0.30]
    realy = [0.15, 0.15, -0.14, -0.14]

    pts_src = np.array([[pointx[0], pointy[0]], [pointx[1], pointy[1]], [pointx[2], pointy[2]], [pointx[3], pointy[3]]])
    pts_dst = np.array([[realx[0], realy[0]], [realx[1], realy[1]], [realx[2], realy[2]], [realx[3], realy[3]]])
    # Calculate Homography
    h, status = cv2.findHomography(pts_src, pts_dst)
    return h


def RANSAC(x, y, n):
    N = 10
    T = 0.02
    n_sample = 30
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
        k = np.floor(n_data * np.random.rand(n_sample));
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
    X = X.T
    # X[0] = X[0] / curv_param
    return X[0]


def LaneDet(frame, lower_rgb, upper_rgb):
    img_color = frame
    crop_lineX_lower = 0.3
    crop_lineX_upper = 0.6
    crop_curvX_lower = 0.5
    crop_curvX_upper = 1
    img_rgb = cv2.cvtColor(img_color, cv2.COLOR_BGR2RGB)
    img_mask = cv2.inRange(img_rgb, lower_rgb, upper_rgb)
    img_mask = (np.array(img_mask))
    coord = np.where(img_mask >= 1)
    coord = np.array([coord[1], coord[0]])
    ones_vec = np.ones((1, np.size(coord[0])))[0]
    coord = np.insert(coord, 2, [ones_vec], axis=0)

    H = Homography()

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
    return poly_coeff_1st, poly_coeff_2nd


def StopLineDet(frame, lower_rgb, upper_rgb):
    img_color = frame
    crop_lineX_lower = 0
    crop_lineX_upper = 1.0
    img_rgb = cv2.cvtColor(img_color, cv2.COLOR_BGR2RGB)
    img_mask = cv2.inRange(img_rgb, lower_rgb, upper_rgb)
    img_mask = (np.array(img_mask))
    coord = np.where(img_mask >= 1)
    coord = np.array([coord[1], coord[0]])
    ones_vec = np.ones((1, np.size(coord[0])))[0]
    coord = np.insert(coord, 2, [ones_vec], axis=0)

    H = Homography()

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

    try:
        # poly_coeff_1st = RANSAC(real_x, real_y, 1)
        poly_coeff_1st = np.polyfit(real_x, real_y, 1)
    except Exception as ex:
        poly_coeff_1st = None
        pass

    return poly_coeff_1st


def display_lines(frame, lines, line_color=(0, 255, 0), line_width=2):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image


key = KeyPoller()
Vx = 0
cnt = 0
r_cnt = 0

with key as poller:
    while True:
        char = poller.poll()

        if char == 'q':
            stop.stop()
            break

        if char == 'a':
            print("\n Cameara Test \n")
            ret, frame = cap.read()
            cv2.imwrite('test' + str(cnt) + '.png', frame)

            LaneDet_Test.frame_test(frame, cnt)
            cnt += 1

        if char == 's':
            print("\n START !! \n")
            start = time.time()

            w = 0.5
            left_line = [0, w / 2]
            right_line = [0, -w / 2]
            left_curv = [0, 0, w / 2]
            right_curv = [0, 0, -w / 2]

            ## pre lane
            left_line_pre = left_line
            right_line_pre = right_line
            left_curv_pre = left_curv
            right_curv_pre = right_curv

            Vx = 0
            dt = 0.1
            Vx_pre = 0
            Ax_pre = 0

            isTarget = 0

            e_a_pre = 0
            e_y_pre = 0
            max_K_pre = 0

            alpha_ey = 0.9
            alpha_ea = 0.9
            alpha_curv = 0.1
            pre_clearance = 200
            clearance = None

            l = 0
            size = 100000
            times = [0 for i in range(size)]
            e_ys = [0 for i in range(size)]
            e_as = [0 for i in range(size)]
            Axs = [0 for i in range(size)]
            Ays = [0 for i in range(size)]
            deltas = [0 for i in range(size)]
            Vxs = [0 for i in range(size)]
            Vxs_des = [0 for i in range(size)]
            dls = [0 for i in range(size)]

            # lower_red = np.array([220, 100, 100], dtype="uint8")
            # upper_red = np.array([255, 250, 255], dtype="uint8")
            # lower_green = np.array([0, 200, 180], dtype="uint8")
            # upper_green = np.array([100, 255, 255], dtype="uint8")
            # lower_white = np.array([240, 240, 240], dtype="uint8")
            # upper_white = np.array([255, 255, 255], dtype="uint8")
            # lower_blue = np.array([0, 140, 230], dtype="uint8")
            # upper_blue = np.array([100, 200, 255], dtype="uint8")

            while True:
                detected_lane_counts = 0

                ret, frame = cap.read()
                #print('ret', ret)

                if (ret):
                    # cv2.imshow('frame_color', frame)


                    #####################
                    ##### RIGHT LANE #####
                    #####################
                    try:
                        right_lane = LaneDet(frame, (235, 100, 110), (255, 180, 255))
                        right_line = right_lane[0]
                    except Exception as ex:
                        # print("error")
                        # print(ex)
                        print(traceback.print_exc())
                        right_line = None
                        pass
                    try:
                        right_curv = right_lane[1]
                    except:
                        right_curv = None
                        pass

                    ######################
                    ##### LEFT LANE #####
                    ######################
                    try:
                        left_lane = LaneDet(frame, (0, 200, 180), (100, 255, 255))
                        left_line = left_lane[0]
                    except Exception as ex:
                        # print("error")
                        # print(ex)
                        # print(traceback.print_exc())
                        left_line = None
                        pass
                    try:
                        left_curv = left_lane[1]
                    except:
                        left_curv = None
                        pass

                    #####################
                    ##### Stop Line #####
                    #####################
                    try:
                        stop_line = StopLineDet(frame, (0, 140, 230), (100, 200, 255))
                        isTarget = 1
                        a = stop_line[0]
                        b = stop_line[1]
                        clearance = min(abs(1 / math.sqrt(1 + a ** 2)), pre_clearance)
                    except:
                        isTarget = 0
                        clearance = None

                    if left_line is not None:
                        detected_lane_counts = detected_lane_counts + 1
                    if right_line is not None:
                        detected_lane_counts = detected_lane_counts + 2

                    if detected_lane_counts == 0:
                        print("BLIND")
                    elif detected_lane_counts == 1:
                        print("Left Lane")
                    elif detected_lane_counts == 2:
                        print("Right Lane")
                    else:
                        print("Both Lanes")

                    if clearance is not None:
                        print("##### STOP #####")
                    else:
                        print("Go")

                ####################
                ##### Planning #####
                ####################
                e_y, e_a = error.err_cal(left_line, right_line, w, e_y_pre, e_a_pre, alpha_ey, alpha_ea)
                max_K = error.max_curv(right_curv, left_curv, max_K_pre, alpha_curv)

                ###################
                ##### CONTROL #####
                ###################
                u = controller.Lateral_control(e_y, e_a)

                Ax, Vx_des, cl_des = controller.Longitudinal_control(Ax_pre=Ax_pre, Vx=Vx, dt=dt, curv_road=max_K,
                                                                     isTarget=isTarget, clearance=clearance)

                Vx = Vx_pre + Ax * dt  # option
                t = time.time()  # loop period
                motor.pwm_ctrl(0.05, Vx, u * 180 / math.pi)

                # print("A: ", A , " B: ", B)
                print("    1/K :      ", round(1 / (max_K + 0.001), 2))
                print("    e_y :      ", round(e_y, 2))
                print("    e_a :      ", round(e_a * 180 / math.pi, 2))
                print("    Steering : ", round(u * 180 / math.pi, 2))
                print("    Vx       : ", round(Vx, 2))
                print("    Ax       : ", round(Ax, 2))
                if clearance is not None:
                    print("    clearance: ", round(clearance, 2))
                print("\n\n\n")

                times[l] = t - start
                e_ys[l] = e_y
                e_as[l] = e_a * 180.0 / math.pi
                Axs[l] = Ax
                Ays[l] = -Vx ** 2 / 0.16 * math.tan(u)
                deltas[l] = u * 180.0 / math.pi
                Vxs[l] = Vx
                Vxs_des[l] = Vx_des
                dls[l] = detected_lane_counts
                l = (l + 1) % size

                max_K_pre = max_K
                left_line_pre = left_line
                right_line_pre = right_line
                left_curv_pre = left_curv
                right_curv_pre = right_curv

                Vx_pre = Vx
                Ax_pre = Ax
                e_y_pre = e_y
                e_a_pre = e_a

                if clearance is not None:
                    pre_clearance = clearance

                end = time.time()
                print("time : ", round(end - start, 4))

                char = poller.poll()

                if char == 'e':
                    print("end")

                    pwm0 = PWM(0)
                    pwm1 = PWM(1)
                    pwm0.enable = False
                    pwm1.enable = False
                    pwm0.unexport()
                    pwm1.unexport()

                    plot.log_plot(l, times, e_ys, e_as, Axs, Ays, deltas, Vxs, Vxs_des, dls, r_cnt)
                    r_cnt += 1
                    break

# if keyboard.is_pressed("a"):
#        pwm0 = PWM(0)
#        pwm1 = PWM(1)
#        pwm0.enable = False
#        pwm1.enable = False
#        pwm0.unexport()
#        pwm1.unexport()

# save log
# if os.path.isdir("./log"):
#    plot.save_log('times', times)
#    plot.save_log('e_ys', e_ys)
#    plot.save_log('e_as', e_as)
#    plot.save_log('Axs', Axs)
#    plot.save_log('Ays', Ays)
#    plot.save_log('deltas', deltas)
#    plot.save_log('Vxs', Vxs)
#    plot.save_log('Vxs_des', Vxs_des)
# else:
#    os.makedirs("./log")

# time.sleep(1)

## motor stop
# pwm0 = PWM(0)
# pwm1 = PWM(1)
# pwm0.enable = False
# pwm1.enable = False
# pwm0.unexport()
# pwm1.unexport()


cap.release()
cv2.destroyAllWindows()