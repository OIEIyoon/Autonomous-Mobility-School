import cv2
import numpy as np
import math
import time

# Perception
from Percep.Camera import Camera

# utils
from utils.keyPoller import KeyPoller
import utils.params as params
from utils.func import init, state_print, backup


# PATH
VIDEO_PATH = "/dev/video"
IMAGE_PATH = "./img"


# TERMINAL INPUT
key = KeyPoller()


# SET CAMERA
webcam = Camera(H = params.H, size = (640, 480))
webcam.get_camera(path = VIDEO_PATH)


if __name__ == "__main__":
    with key as poller:
        while True:
            char = poller.poll()

            if char == 'q': # program quit
                print("\n Program Quit \n")
                break


            if char == 'c': # capture img
                print("\n Camera Test \n")
                webcam.capture(IMAGE_PATH + '/capture')


            if char == 's': # start driving
                print("\n START !! \n")

                # init
                ismotor = 1
                K, car, lane, stopline, error, controller, info, motor, dt = init(ismotor)
                start = time.time()

                while True:
                    t = time.time()
                    webcam.read()


                    ################################################################################
                    ################################### 1. 인 지 ###################################
                    ################################################################################
                    if webcam.ret == True:
                        # right, left lane detection
                        lane.get_lane(webcam.frame, alpha_curv = 0.1,
                                      left_lower = (0, 200, 180), left_upper = (100, 255, 255), right_lower = (235, 100, 110), right_upper = (255, 180, 255),
                                      crop_lineX_lower = 0.3, crop_lineX_upper = 0.6, crop_curvX_lower = 0.5, crop_curvX_upper = 1)

                        if lane.detected_lane_counts == 0:
                            print("BLIND")
                        elif lane.detected_lane_counts == 1:
                            print("Left Lane")
                        elif lane.detected_lane_counts == 2:
                            print("Right Lane")
                        else:
                            print("Both Lanes")

                        # stop line detection
                        stopline.StopLineDet(webcam.frame, lower_rgb = (0, 140, 230), upper_rgb = (100, 200, 255),
                                             crop_lineX_lower = 0.3, crop_lineX_upper = 0.6)

                        if stopline.clearance is not None:
                            print("##### STOP #####")
                        else:
                            print("Go")
                    ################################################################################
                    ################################################################################




                    ################################################################################
                    ################################ 2. 판 단, 제 어 ###############################
                    ################################################################################
                    error.err_cal(lane.left_lane.left_line, lane.right_lane.right_line, w=0.5)

                    car.u = controller.Lateral_control(error.e_y, error.e_a)

                    car.Ax, Vx_des, cl_des = controller.Longitudinal_control(Ax_pre= car.Ax_pre, Vx= car.Vx, dt = dt, curv_road= lane.max_K,
                                            isTarget= stopline.isTarget, clearance= stopline.clearance)

                    car.Vx = car.Vx_pre + car.Ax * dt # dt = 0.1
                    #################################################################################
                    #################################################################################



                    # output
                    if ismotor == 1:
                        motor.pwm_ctrl(0.05, car.Vx, car.u * 180 / math.pi)

                    # print
                    state_print(start, lane, error, car, stopline)

                    # save and backup
                    backup(start, Vx_des, info, lane, stopline, car, error)

                    # dt update
                    dt = time.time() - t

                    # check end
                    char = poller.poll()
                    if char == 'e': # end driving
                        print("end driving")

                        if ismotor == 1:
                            motor.kill()
                            ismotor = 0

                        info.save(path='./img/result', K = K)
                        break


webcam.release()
cv2.destroyAllWindows()