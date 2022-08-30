import cv2
import math
import time
import traceback
import os

from Percep.Camera import Camera

from utils.func import init, state_print, backup
from Percep.utils.func import Homography

if __name__ == "__main__":
    # Variable
    pointx = [177, 464, 605, 31]
    pointy = [299, 300, 394, 394]
    realx = [0.6, 0.6, 0.3, 0.3]
    realy = [0.15, -0.15, -0.15, 0.15]
    H = Homography(pointx, pointy, realx, realy)


    webcam = Camera(H, crop_lineX_lower=0.2, crop_lineX_upper=0.4, crop_curvX_lower=0.3, crop_curvX_upper=0.6,
                    lower_green=[90, 200, 90], upper_green=[120, 255, 120],
                    lower_red=[200, 90, 90], upper_red=[255, 120, 120],
                    lower_blue=[90, 90, 200], upper_blue=[120, 120, 255])
    webcam.get_test_video("./resource/test_video/test_driving1.mp4")


    # init
    ismotor = 0
    K, car, lane, stopline, error, controller, info, motor, dt = init(ismotor)
    start = time.time()


    ############################################
    print("get frames")
    prev_time = 0
    video_len = 25
    FPS = 10
    frames = [0 for i in range(10000)]
    cnt = 0
    while True:
        webcam.read()
        if webcam.ret == False:
            break
        elif cnt%10 == 0:
            webcam.capture(path = './img/capture')

        if cv2.waitKey(1) > 0:
            break

        current_time = time.time() - prev_time
        if (webcam.ret is True) and (current_time > 1. / FPS):
            frames[cnt] = webcam.frame
            cnt += 1
            print(cnt)
            prev_time = time.time()
    start = time.time()
    ################################################

    print("\n START \n")
    for i in range(cnt):
        if not os.path.isdir("./img/frame"):
            print("No directory, create directory")
            os.makedirs('./img/frame')
        cv2.imwrite("./img/frame/frame" + str(i) + ".png", frames[i])
        #webcam.frame = frames[i]
        #webcam.capture('./img')

        t = time.time()

        if True:
            # right, left lane detection
            lane.get_lane(frames[i], alpha_curv = 0.1,
                        crop_lineX_lower=0.2, crop_lineX_upper=0.4, crop_curvX_lower=0.3, crop_curvX_upper=0.6)
            if lane.detected_lane_counts == 0:
                print("BLIND")
            elif lane.detected_lane_counts == 1:
                print("Left Lane")
            elif lane.detected_lane_counts == 2:
                print("Right Lane")
            else:
                print("Both Lanes")

            # stop line detection
            stopline.StopLineDet(frames[i],
                        crop_lineX_lower=0.2, crop_lineX_upper=0.4)
            if stopline.clearance is not None:
                print("##### STOP #####")
            else:
                print("Go")

        # planning
        error.err_cal(lane.left_lane.left_line, lane.right_lane.right_line, w = 0.3)

        # control
        car.u = controller.Lateral_control(error.e_y, error.e_a)
        car.Ax, Vx_des, cl_des = controller.Longitudinal_control(Ax_pre= car.Ax_pre, Vx= car.Vx, dt = dt, curv_road= lane.max_K,
                                                             isTarget= stopline.isTarget, clearance= stopline.clearance)
        car.Vx = car.Vx_pre + car.Ax * dt # dt = 0.1

        # output
        if ismotor == 1:
            motor.pwm_ctrl(0.05, car.Vx, car.u * 180 / math.pi)

        # print
        state_print(start, lane, error, car, stopline)

        # save and backup
        backup(start, Vx_des, info, lane, stopline, car, error)

        # dt update
        dt = time.time() - t

    info.save(path = './img/result', K = K)

    webcam.release()
    cv2.destroyAllWindows()
