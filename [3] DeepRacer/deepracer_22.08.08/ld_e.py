from pwm import PWM
import cv2
import numpy as np
import matplotlib.pyplot as plt

import error
import controller
import plot
import math
import time
import motor
import traceback
from keyPoller import KeyPoller

cap = cv2.VideoCapture("/dev/video1")


def detect_blue(frame):
    blur = cv2.GaussianBlur(frame, (0, 0), 3)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    #gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    # filter for blue lane lines
    lower_blue = np.array([93, 94, 230], dtype="uint8")
    upper_blue = np.array([100, 255, 255], dtype="uint8")
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    # mask_white = cv2.inRange(gray, 200, 255)
    # mask_bu = cv2.bitwise_or(mask_white, mask_blue)
    # mask_bu = cv2.bitwise_or(mask_blue)
    # mask_bu_image = cv2.bitwise_and(gray, mask_bu)

    # detect edges
    edges = cv2.Canny(mask_blue, 800, 800)
    
    return edges

def detect_red(frame):
    #blur = cv2.GaussianBlur(frame, (0, 0), 3)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    #gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    # filter for blue lane lines
    lower_red = np.array([220, 100, 100], dtype="uint8")
    upper_red = np.array([255, 200, 200], dtype="uint8")
    mask_red = cv2.inRange(rgb, lower_red, upper_red)
    # mask_white = cv2.inRange(gray, 200, 255)
    # mask_bu = cv2.bitwise_or(mask_white, mask_blue)
    # mask_bu = cv2.bitwise_or(mask_blue)
    # mask_bu_image = cv2.bitwise_and(gray, mask_bu)

    # detect edges
    edges = cv2.Canny(mask_red, 800, 800)

    return edges



# lab blue value [95, 200, 180] / [110, 255, 230]
# lab green value [70, 90, 170] / [90, 255, 245]
# practice red value [0, 100, 230] / [190, 140, 255]
# practice green value [80, 175, 225] / [90, 255, 255]
# practice blue value [95, 240, 240] / [110, 255, 255]


def detect_green(frame):
    blur = cv2.GaussianBlur(frame, (0, 0), 3)
    rgb = cv2.cvtColor(blur, cv2.COLOR_BGR2RGB)
    #gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    # filter for blue lane lines
    lower_red = np.array([0, 200, 180], dtype="uint8")
    upper_red = np.array([100, 255, 255], dtype="uint8")
    mask_red = cv2.inRange(rgb, lower_red, upper_red)
    # mask_white = cv2.inRange(gray, 200, 255)
    # mask_bu = cv2.bitwise_or(mask_white, mask_blue)
    # mask_bu = cv2.bitwise_or(mask_blue)
    # mask_bu_image = cv2.bitwise_and(gray, mask_bu)

    # detect edges
    edges = cv2.Canny(mask_red, 800, 800)

    return edges

     
    return edges

def detect_green_HSV(frame):
    blur = cv2.GaussianBlur(frame, (0, 0), 3)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    #hsv = cv2.GaussianBlur(frame, (0, 0), 4)
    #gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    # filter for red lane lines
    lower_red = np.array([82, 120, 210], dtype="uint8")
    upper_red = np.array([91, 255, 255], dtype="uint8")
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    # mask_white = cv2.inRange(gray, 200, 255)
    # mask_re = cv2.bitwise_or(mask_white, mask_red)
    # mask_re = cv2.bitwise_or(mask_red)
    # mask_re_image = cv2.bitwise_and(gray, mask_re)

    # detect edges
    edges = cv2.Canny(mask_red, 800, 800)
    #dil_kernel = np.ones((3, 3), np.uint8)
    #dilation = cv2.dilate(edges, dil_kernel, 1)
     
    return edges


# img_result = detect_red(frame)
# plt.imshow(img_result, cmap='gray')
# plt.show()

def region_of_interest_first(edges):
    height, width = edges.shape
    mask = np.zeros_like(edges)

    # only focus bottom half of the screen
    polygon = np.array([[
        (0, height),
        (width, height),
        (width, height * 1 / 4),
        (0, height * 1 / 4),
    ]], np.int32)  

    cv2.fillPoly(mask, polygon, 255)
    cropped_edges = cv2.bitwise_and(edges, mask)

    return cropped_edges

def region_of_interest_second(edges):
    height, width = edges.shape
    mask = np.zeros_like(edges)

    # only focus bottom half of the screen
    polygon = np.array([[
        (0, height), # * 1/4
        (width, height), # * 1/4
        (width, height * 1 / 4), # * 1/2
        (0, height * 1 / 4),# * 1/2
    ]], np.int32)  

    cv2.fillPoly(mask, polygon, 255)
    cropped_edges = cv2.bitwise_and(edges, mask)

    return cropped_edges


def detect_line_segments(cropped_edges):
    # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
    rho = 1  # distance precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
    min_threshold = 10  # minimal of votes
    line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold,
                                    np.array([]), minLineLength=8, maxLineGap=4)
    line_segments = np.mean(line_segments, axis =0)
    print("line_segments", line_segments)
    return line_segments


# def average_slope_intercept(frame, line_segments):
#     """
#     This function combines line segments into one or two lane lines
#     If all line slopes are < 0: then we only have detected left lane
#     If all line slopes are > 0: then we only have detected right lane
#     """
#     lane_lines = []
#     if line_segments is None:
#         # lane_lines = [[None for i in range(20)] for j in range(2)]
#         return lane_lines

#     lane_fit = []
#     for line_segment in line_segments:
#         for x1, y1, x2, y2 in line_segment:
#             if x1 == x2:
#                 continue
#             fit = np.polyfit((x1, x2), (y1, y2), 1)
#             slope = fit[0]
#             intercept = fit[1]
#             lane_fit.append((slope, intercept))

#     lane_fit_average = np.average(lane_fit, axis=0)
#     if len(lane_fit) > 0:
#         lane_lines.append(make_points(frame, lane_fit_average))

#     return lane_lines


# def average_slope_intercept(frame, line_segments):
#    for line_segment in line_segments:
#        for x1, y1, x2, y2 in line_segment:


# def make_points(frame, line):
#     height, width = frame.shape
#     slope, intercept = line
#     y1 = height  # bottom of the frame
#     y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

#     # bound the coordinates within the frame
#     x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
#     x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
#     return [[x1, y1, x2, y2]]


def Homography():
    # 바닥을 기준으로 한 point x/y 는 image pixel 상 2차원 좌표, real x/y 는 실제 트랙 위 3차원 좌표(z=0) 4개를 넣어주면 호모그래피 행렬 H 값 나옴.
    pointx = [62, 124, 466, 542]
    pointy = [346, 277, 272, 349]
    realx = [0.31, 0.455, 0.455, 0.30]
    realy = [0.15, 0.15, -0.14, -0.14]

    pts_src = np.array([[pointx[0], pointy[0]], [pointx[1], pointy[1]], [pointx[2], pointy[2]], [pointx[3], pointy[3]]])
    pts_dst = np.array([[realx[0], realy[0]], [realx[1], realy[1]], [realx[2], realy[2]], [realx[3], realy[3]]])
    # Calculate Homography
    h, status = cv2.findHomography(pts_src, pts_dst)
    return h

def frame_points_f(points):
    a = np.polyfit([points[0][0], points[0][2]], [points[0][1], points[0][3]], 1)
    b = np.linspace(points[0][0], points[0][2], 20)
    #print("b: ", b)
    point = []
    real = []
    realx = []
    realy = []
    H = Homography()
    
    lane_points = []
    i = 0
    for x in b:
        i = a[0] * x + a[1] 
        point.append([x, i, 1])
    for y in range(20):
        real.append(H @ (np.transpose(point[y])))
    for z in range(20):
        realx.append((np.transpose(real[z]))[0] / (np.transpose(real[z]))[2])
        realy.append((np.transpose(real[z]))[1] / (np.transpose(real[z]))[2])
    # for z in range(20):
    lane_points = [realx, realy]
    print("lane_points: ", lane_points)
    return lane_points

def frame_points_s(points_first, points_second):
    a = np.polyfit([points_first[0][0], points_first[0][2], points_second[0][2]], [points_first[0][1], points_first[0][3], points_second[0][3]], 2)
    b = np.linspace(points_first[0][0], points_second[0][2], 20)
    #print("b: ", b)
    point = []
    real = []
    realx = []
    realy = []
    H = Homography()
    
    lane_points = []
    i = 0
    for x in b:
        i = a[0] * (x ** 2) + a[1] * x + a[2]
        point.append([x, i, 1])
    for y in range(20):
        real.append(H @ (np.transpose(point[y])))
    for z in range(20):
        realx.append((np.transpose(real[z]))[0] / (np.transpose(real[z]))[2])
        realy.append((np.transpose(real[z]))[1] / (np.transpose(real[z]))[2])
    # for z in range(20):
    lane_points = [realx, realy]
    #print("lane_points: ", lane_points)
    return lane_points


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


with key as poller:
    while True:
        char = poller.poll()
        
        if char == 's':
            print("start")
            start = time.time()
            
            ## pre lane
            points = 20
            pre_coef1 = [0, 0, 0.2]
            pre_coef2 = [0, 0, 0.2]

            pre_left_lane = [[0 for i in range(points)] for j in range(2)]
            pre_right_lane = [[0 for i in range(points)] for j in range(2)]
            
            pre_left_lane_pos = [[0 for i in range(points)] for j in range(2)]
            pre_right_lane_pos = [[0 for i in range(points)] for j in range(2)]

            pre_curv = 0
            
            left_lane = [[i for i in range(points)] for j in range(2)]
            right_lane = [[i for i in range(points)] for j in range(2)]

            left_pos = [[i for i in range(points)] for j in range(2)]
            right_pos = [[i for i in range(points)] for j in range(2)]
        
            Vx = 0
            dt = 0.1

            Vx_pre = 0
            Ax_pre = 0
            
            e_a_pre = 0
            e_y_pre = 0
        
            alpha = 1
        
            w = -0.2
        
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

            while True:
                check = 0
                detected_lane_counts = 0
                
                ret, frame = cap.read()
                print('ret', ret)
                if (ret):
                    # cv2.imshow('frame_color', frame)
                    try:
                        img_green_f = region_of_interest_first(detect_green_HSV(frame))
                        img_green_s = region_of_interest_second(detect_green_HSV(frame))
                        right_lane = frame_points_f(detect_line_segments(img_green_f))
                        right_pos = frame_points_s(detect_line_segments(img_green_f), detect_line_segments(img_green_s))
                        print("right_lane detected")
                        plt.scatter(right_pos[0], right_pos[1])
                        #lane_lines = average_slope_intercept(img_green, detect_line_segments(img_green))
                        #lane_lines_image = display_lines(frame, lane_lines)
                        #cv2.imshow("lines", lane_lines_image)
                        #if cv2.waitKey(1) & 0xFF == ord('q'):
                            #break
                     
                        detected_lane_counts = 1
                        check = 1
                    except Exception as ex:
                        # print("error")
                        #print(ex)
                        #print(traceback.print_exc())
                        right_pos[0][0] = None
                        pass
                        
                
                    try:
                        img_blue_f = region_of_interest_first(detect_red(frame))
                        img_blue_s = region_of_interest_second(detect_red(frame))
                        left_lane = frame_points_f(detect_line_segments(img_blue_f))
                        left_pos = frame_points_s(detect_line_segments(img_blue_f), detect_line_segments(img_blue_s))
                        print("left_lane detected")
                        plt.scatter(left_pos[0], left_pos[1])
                        #lane_lines = average_slope_intercept(img_blue, detect_line_segments(img_blue))
                        #lane_lines_image = display_lines(frame, lane_lines)
                        #cv2.imshow("lines", lane_lines_image)
                        #if cv2.waitKey(1) & 0xFF == ord('q'):s
                            #break
                        #print(left_pos)
                        #print("r: ", right_pos)
                        detected_lane_counts = 2
                        if check == 1:
                            detected_lane_counts = 3
                    except Exception as ex:
                        # print("error")
                        #print(ex)
                        print(traceback.print_exc())
                        left_pos[0][0] = None
                        pass
                 
                plt.axis('equal')
                plt.show()
                #left_pos[0][0] = None
            
                coef1 = error.route(pre_coef1, pre_left_lane, pre_right_lane, left_lane, right_lane, alpha=alpha,
                       w=w)
                coef2 = error.route(pre_coef2, pre_left_lane_pos, pre_right_lane_pos, left_pos, right_pos, alpha=alpha,
                       w=w)
                A, B, e_y, e_a = error.get_error(coef1)
                A, B, _, _ = error.get_error(coef2)
                
                
                e_y = e_y * alpha + (1-alpha) * e_y_pre
                e_a = e_a * alpha + (1-alpha) * e_a_pre
                
                prev_k, traj_max_k = error.curvature(coef2, A, B, 1)

                curv_road = traj_max_k * alpha + (1-alpha) * pre_curv ## curv filtering

                u = controller.Lateral_control(e_y, e_a)

                Ax, Vx_des, cl_des = controller.Longitudinal_control(Ax_pre=Ax_pre, Vx=Vx, dt=dt, curv_road=curv_road,
                                                         isTarget=0, clearance=None)
                Vx = Vx_pre + Ax * dt  # option

                t = time.time()  # one loop time

                # print("A: ", A , " B: ", B)
                print("prev_curvature: ", prev_k, " traj_curvature: ", curv_road, " e_y: ", e_y, " e_a: ", e_a * 180 / math.pi)
                print("steer_angle: ", u* 180 / math.pi)
                print("Ax: ", Ax, "Vx: ", Vx, "Vx_des: ", Vx_des, "cl_des: ", cl_des, '\n')

                motor.pwm_ctrl(0.05, Vx ,u * 180 / math.pi)

                
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
                

                pre_curv = curv_road
                pre_coef1 = coef1
                pre_coef2 = coef2
                pre_left_lane = left_lane
                pre_right_lane = right_lane
                pre_left_lane_pos = left_pos
                pre_right_lane_pos = right_pos
                #Vx_pre = Vx
                Ax_pre = Ax
                e_y_pre = e_y
                e_a_pre = e_a

                end = time.time()
                print("time : ", end - start)

                char = poller.poll()

                if char == 'e':
                    print("end")
                    
                    pwm0 = PWM(0)
                    pwm1 = PWM(1)
                    pwm0.enable = False
                    pwm1.enable = False
                    pwm0.unexport()
                    pwm1.unexport()                     
                    
                    plot.log_plot(l, times, e_ys, e_as, Axs, Ays, deltas, Vxs, Vxs_des, dls, left_pos, right_pos)
                         
                    break


#    if keyboard.is_pressed("a"):
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

