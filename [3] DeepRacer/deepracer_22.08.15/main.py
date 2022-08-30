import cv2
import math
import time

# Perception
from Percep.Camera import Camera

# utils
from utils.keyPoller import KeyPoller
import utils.params as params
from utils.func import init, state_print, backup, init_rgb_range, help_message, read_rgb_range

# PATH
VIDEO_PATH = "/dev/video"
IMAGE_PATH = "/home/deepracer/Desktop/img"

# TERMINAL INPUT
key = KeyPoller()

# SET CAMERA
webcam = Camera(H=params.H, size=(640, 480))
webcam.get_camera(path=VIDEO_PATH)

# rgb range
red_lower, red_upper, green_lower, green_upper, blue_lower, blue_upper = read_rgb_range(
                    "/home/deepracer/Desktop/rgb_range.txt")


if __name__ == "__main__":
    help_message()

    with key as poller:
        while True:
            char = poller.poll()

            if char == 'h':  # show help message
                help_message()

            if char == 'w':
                red_lower, red_upper, green_lower, green_upper, blue_lower, blue_upper = get_rgb_range()
                help_message()
            """
            if char == 'i':  # set rgb range
                webcam.release()

                init_rgb_range(VIDEO_PATH, IMAGE_PATH, "/home/deepracer/Desktop")
                red_lower, red_upper, green_lower, green_upper, blue_lower, blue_upper = read_rgb_range(
                    "/home/deepracer/Desktop/rgb_range.txt")

                webcam = Camera(H=params.H, size=(640, 480),
                                lower_green=list(green_lower), upper_green=list(green_upper),
                                lower_red=list(red_lower), upper_red=list(red_upper),
                                lower_blue=list(blue_lower), upper_blue=list(blue_upper))
                webcam.get_camera(path=VIDEO_PATH)
                help_message()
            """
            if char == 'r':  # re-get camera
                print("re-get camera")
                webcam.release()

                red_lower, red_upper, green_lower, green_upper, blue_lower, blue_upper = read_rgb_range(
                    "/home/deepracer/Desktop/rgb_range.txt")
                webcam = Camera(H=params.H, size=(640, 480),
                                lower_green=list(green_lower), upper_green=list(green_upper),
                                lower_red=list(red_lower), upper_red=list(red_upper),
                                lower_blue=list(blue_lower), upper_blue=list(blue_upper))
                webcam.get_camera(path=VIDEO_PATH)
                help_message()

            if char == 'q':  # program quit
                print("\n Program Quit \n")
                break

            if char == 'c':  # capture img
                webcam.capture(IMAGE_PATH + '/capture')
                help_message()

            if char == 's':  # start driving
                print("\n START !! \n")

                # init
                ismotor = 1
                K, car, lane, stopline, error, controller, info, motor, dt = init(ismotor, left_lower=green_lower,
                                                                                  left_upper=green_upper,
                                                                                  right_lower=red_lower,
                                                                                  right_upper=red_upper,
                                                                                  lower_rgb=blue_lower,
                                                                                  upper_rgb=blue_upper)
                start = time.time()

                while True:
                    t = time.time()
                    webcam.read()

                    ################################################################################
                    ################################### 1. 인 지 ###################################
                    ################################################################################
                    if webcam.ret == True:
                        # right, left lane detection
                        lane.get_lane(webcam.frame, alpha_curv=0.1,
                                      crop_lineX_lower=0.2, crop_lineX_upper=0.4, crop_curvX_lower=0.3,
                                      crop_curvX_upper=0.6, y1=0.3, y2=0.4)

                        if lane.detected_lane_counts == 0:
                            print("BLIND")
                        elif lane.detected_lane_counts == 1:
                            print("Left Lane")
                        elif lane.detected_lane_counts == 2:
                            print("Right Lane")
                        else:
                            print("Both Lanes")

                        # stop line detection
                        stopline.StopLineDet(webcam.frame,
                                             crop_lineX_lower=0, crop_lineX_upper=1, y1=0.3)  # 0.3 0.6

                        if stopline.clearance is not None:
                            print("##### STOP #####")
                        else:
                            print("Go")
                    ################################################################################
                    ################################################################################




                    ################################################################################
                    ################################ 2. 판 단, 제 어 ###############################
                    ################################################################################
                    error.err_cal(lane.left_lane.left_line, lane.right_lane.right_line, w=0.6)
                    error.e_a = error.e_a - 2 / 180  * math.pi

                    car.u = controller.Lateral_control(error.e_y, error.e_a, dt)

                    car.Ax, Vx_des, cl_des, car.BSC, car.ax_nom = controller.Longitudinal_Control(Ax_pre=car.Ax_pre, Vx=car.Vx, dt=dt,
                                                                             curv_road=lane.max_K,
                                                                             isTarget=stopline.isTarget,
                                                                             clearance=stopline.clearance,
                                                                                          BSC_pre = car.BSC_pre, ax_nom_pre = car.ax_nom_pre)

                    car.Vx = car.Vx_pre + car.Ax * dt  # dt = 0.1
                    #################################################################################
                    #################################################################################

                    # output
                    if ismotor == 1:
                        print("\n output \n")
                        motor.pwm_ctrl(0.05, car.Vx, car.u * 180 / math.pi)

                    # print
                    state_print(start, lane, error, car, stopline)

                    # save and backup
                    backup(start, Vx_des, info, lane, stopline, car, error)

                    # dt update
                    dt = time.time() - t

                    # check end
                    char = poller.poll()
                    if char == 'e':  # end driving
                        print("\n end driving \n")

                        if ismotor == 1:
                            motor.stop()
                            ismotor = 0

                        info.save(path=IMAGE_PATH + '/result', K=K)
                        help_message()
                        break

webcam.release()
cv2.destroyAllWindows()
