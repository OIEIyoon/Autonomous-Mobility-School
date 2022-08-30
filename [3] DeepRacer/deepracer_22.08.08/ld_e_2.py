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
import stop
from keyPoller import KeyPoller


import LaneDet_Test

cap = cv2.VideoCapture("/dev/video2")


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



def LaneDet(frame, lower_rgb, upper_rgb):
    img_color = frame
    height, width, _ = img_color.shape
    img_rgb = cv2.cvtColor(img_color, cv2.COLOR_BGR2RGB)
    # lower_red = np.array([220, 100, 100], dtype="uint8")
    # upper_red = np.array([255, 250, 255], dtype="uint8")
    # lower_green = np.array([0, 200, 180], dtype="uint8")
    # upper_green = np.array([100, 255, 255], dtype="uint8")
    # lower_rgb = np.array([240, 240, 240], dtype="uint8")
    # upper_rgb = np.array([255, 255, 255], dtype="uint8")
    img_mask = cv2.inRange(img_rgb, lower_rgb, upper_rgb)

    img_mask = (np.array(img_mask))
    coord = np.where(img_mask>=1)
    coord = np.array([coord[1], coord[0]])
    ones_vec = np.ones((1,np.size(coord[0])))[0]
    coord = np.insert(coord, 2, [ones_vec], axis = 0)

    H = Homography()

    homo = np.dot(H, coord)
    lane_points = [homo[0] / homo[2], homo[1] / homo[2]]

    idx_x1, idx_x2 = np.where(lane_points[0]>0), np.where(lane_points[0]<0.5)
    idx_x = np.intersect1d(idx_x1, idx_x2)
    idx_y1, idx_y2 = np.where(lane_points[1]>-0.5), np.where(lane_points[1]<0.5)
    idx_y = np.intersect1d(idx_y1, idx_y2)
    idx = np.intersect1d(idx_x, idx_y)
    real_x = lane_points[0][idx]
    real_y = lane_points[1][idx]

    idx_x1_curv, idx_x2_curv = np.where(lane_points[0]>=0.7), np.where(lane_points[0]<1)
    idx_x_curv = np.intersect1d(idx_x1_curv, idx_x2_curv)
    idx_y1_curv, idx_y2_curv = np.where(lane_points[1]>-0.5), np.where(lane_points[1]<0.5)
    idx_y_curv = np.intersect1d(idx_y1_curv, idx_y2_curv)
    idx_curv = np.intersect1d(idx_x_curv, idx_y_curv)
    real_x_curv = lane_points[0][idx_curv]
    real_y_curv = lane_points[1][idx_curv]

    xq = np.linspace(0,1,50)
    try:
        poly_coeff_1st = np.polyfit(real_x, real_y, 1)
        yq = np.polyval(poly_coeff_1st, xq)
    except Exception as ex:
        #print("error")
        #print(ex)
        #print(traceback.print_exc())
        poly_coeff_1st = None
        yq = None
        pass

    try:
        poly_coeff_2nd = np.polyfit(real_x_curv, real_y_curv, 2)
        yq2 = np.polyval(poly_coeff_2nd, xq)
    except Exception as ex:
        #print("error")
        #print(ex)
        #print(traceback.print_exc())
        poly_coeff_2nd = None
        yq2 = None
        pass

    return poly_coeff_1st, poly_coeff_2nd


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

with key as poller:
    while True:
        char = poller.poll()
        
        if char == 'q':
            stop.stop()
            break
        
        if char == 'a':
            print("cameara test")
            ret, frame = cap.read()
            cv2.imwrite('test' + str(cnt) + '.png', frame)
            
            LaneDet_Test.frame_test(frame, cnt)
        
        if char == 's':
            print("start")
            start = time.time()
            
            left_line = [0, 0]
            right_line = [0, 0]
            
            left_curv = [0, 0, 0]
            right_curv = [0, 0, 0]
            
            ## pre lane
            points = 20
            pre_coef1 = [0, 0, 0.2]
            pre_coef2 = [0, 0, 0.2]

            pre_left_line = [0, 0]
            pre_right_line = [0, 0]
            
            pre_left_curv = [0, 0, 0]
            pre_right_curv = [0, 0, 0]

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
            beta = 0.1
        
            w = 0.6
        
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
                detected_lane_counts = 0
                
                ret, frame = cap.read()
                print('ret', ret)
    
                if (ret):
                    # cv2.imshow('frame_color', frame)
                    try:
                        right_lane = LaneDet(frame, (0, 200, 180), (100, 255, 255))
                        right_line = right_lane[0]
                        print("right_lane_coef: ", right_lane)
                        #print("RIGHT_lane detected")
                        detected_lane_counts = detected_lane_counts + 1
                    except Exception as ex:
                        # print("error")
                        #print(ex)
                        #print(traceback.print_exc())
                        right_line = None
                        pass
                    try: 
                        right_curv = right_lane[1]
                    except:
                        right_curv = None
                        pass
                        
                
                    try:
                        left_lane = LaneDet(frame, (240, 240, 240), (255, 255, 255))
                        left_line = left_lane[0]
                        print("left_lane_coef: ", left_lane)
                        #print("LEFT_lane detected")
                        detected_lane_counts = detected_lane_counts + 2
                    except Exception as ex:
                        # print("error")
                        #print(ex)
                        print(traceback.print_exc())
                        left_line = None
                        pass
                    try: 
                        left_curv = left_lane[1]
                    except:
                        left_curv = None
                        pass
                 

                e_y, e_a = error.route(right_line, left_line, w)
                if e_y is not None and e_a is not None:
                    e_y = e_y * alpha + (1-alpha) * e_y_pre
                    e_a = e_a * alpha + (1-alpha) * e_a_pre
                else:
                    e_y = e_y_pre
                    e_a = e_a_pre
                e_y = e_y_pre + min(0.2, max(-0.2, e_y-e_y_pre))
                e_a = e_a_pre + min(10*math.pi/180, max(-10*math.pi/180, e_a-e_a_pre))
                
                e_y = min(0.4, max(-0.4, e_y))
                e_a = min(50*math.pi/180, max(-50*math.pi/180, e_a))
                    
                max_K = error.max_curv(right_curv, left_curv)
                if max_K is not None:
                    max_K = max_K * beta + (1-beta) * pre_curv
                else:
                    max_K = pre_curv
                

                # prev_k, traj_max_k = error.curvature(coef2, A, B, 1)
                # curv_road = traj_max_k * alpha + (1-alpha) * pre_curv ## curv filtering

                u = controller.Lateral_control(e_y, e_a)

                Ax, Vx_des, cl_des = controller.Longitudinal_control(Ax_pre=Ax_pre, Vx=Vx, dt=dt, curv_road=max_K,
                                                         isTarget=0, clearance=None)
                Vx = Vx_pre + Ax * dt  # option

                t = time.time()  # one loop time

                # print("A: ", A , " B: ", B)
                print("    curv_radi: ", round(1/max_K,2))
                print("    e_y: ", round(e_y,2))
                print("    e_a: ", round(e_a * 180 / math.pi,2))
                print("    steer_angle: ", round(u* 180 / math.pi,2))
#                 print("Ax: ", Ax, "Vx: ", Vx, "Vx_des: ", Vx_des, "cl_des: ", cl_des, '\n')

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
                

                pre_curv = max_K
                pre_left_line = left_line 
                pre_right_line = right_line 
                pre_left_curv = left_curv
                pre_right_curv = right_curv

                #Vx_pre = Vx
                Ax_pre = Ax
                e_y_pre = e_y
                e_a_pre = e_a

                end = time.time()
                print("time : ", round(end - start,4))

                char = poller.poll()

                if char == 'e':
                    print("end")
                    
                    pwm0 = PWM(0)
                    pwm1 = PWM(1)
                    pwm0.enable = False
                    pwm1.enable = False
                    pwm0.unexport()
                    pwm1.unexport()                     
                    
                    plot.log_plot(l, times, e_ys, e_as, Axs, Ays, deltas, Vxs, Vxs_des, dls)
                         
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

