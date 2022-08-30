import cv2
import math
import time
import traceback

# Perception
from Percep.Camera import Camera
from Percep.Det import Lane, StopLine

# Planning Control
from Plan.Planning import Error
from Plan.Controller import Controller

# Pwm
from Motor.motor import Motor

# utils
# from utils.keyPoller import KeyPoller
from utils.params import Car, Info



if __name__ == "__main__":
    # Variable
    webcam = Camera(lower_green=[90, 200, 90], upper_green=[120, 255, 120],
                    lower_red=[200, 90, 90], upper_red=[255, 120, 120],
                    lower_blue=[90, 90, 200], upper_blue=[120, 120, 255])
    webcam.get_test_video("C:/Users/oni/PycharmProjects/deepracer/resource/test_video/test_driving1.mp4")

    time_rate = 1
    end_time = 10

    #motor = Motor(vel_max=1.5, dt=0.01)
    ismotor = 0  # "motor on" flag

    print("\n START !! \n")
    start = time.time()

    # init
    car = Car()

    lane = Lane(w = 0.5, left_lower = (90, 200, 90), left_upper = (120, 255, 120),
                 right_lower = (200, 90, 90), right_upper = (255, 120, 120))
    stopline = StopLine(2, 0.9, (90, 90, 200), (120, 120, 255))

    error = Error(alpha_ey = 0.9, alpha_ea = 0.9)
    controller = Controller(steer_angle_max = 30, Vx_max = 0.4, c_min = 0.1, tau = 1.4, Ay_max = 0.7)
    info = Info(size = 100000)

    ismotor = 0  # motor flag off
    dt = 0.1

    cmp = time_rate

    while True:
        t = time.time()
        webcam.read()

        if webcam.ret == True:
            # right, left lane detection
            lane.get_lane(webcam.frame, alpha_curv = 0.1)
            if lane.detected_lane_counts == 0:
                print("BLIND")
            elif lane.detected_lane_counts == 1:
                print("Left Lane")
            elif lane.detected_lane_counts == 2:
                print("Right Lane")
            else:
                print("Both Lanes")

            # stop line detection
            stopline.StopLineDet(webcam.frame)
            if stopline.clearance is not None:
                print("##### STOP #####")
            else:
                print("Go")
        else:
            info.save("./img")
            break

        # planning
        error.err_cal(lane.left_lane.left_line, lane.right_lane.right_line, w = 0.5)

        # control
        car.u = controller.Lateral_control(error.e_y, error.e_a, k_a = 4, k_y = 2.5)
        car.Ax, Vx_des, cl_des = controller.Longitudinal_control(Ax_pre= car.Ax_pre, Vx= car.Vx, dt = dt, curv_road= lane.max_K,
                                                             isTarget= stopline.isTarget, clearance= stopline.clearance)
        car.Vx = car.Vx_pre + car.Ax * dt # dt = 0.1

        # output
        #if ismotor == 1:
            #motor.pwm_ctrl(0.05, car.Vx, car.u * 180 / math.pi)

        # print
        print("    1/K :      ", round(1 / (lane.max_K + 0.001), 2))
        print("    e_y :      ", round(error.e_y, 2))
        print("    e_a :      ", round(error.e_a * 180 / math.pi, 2))
        print("    Steering : ", round(car.u * 180 / math.pi, 2))
        print("    Vx       : ", round(car.Vx, 2))
        print("    Ax       : ", round(car.Ax, 2))
        if stopline.clearance is not None:
            print("    clearance: ", round(stopline.clearance, 2))
        print("\n\n\n")
        print("running time : ", round(time.time() - start, 4))


        if time.time() - start >= cmp:
            webcam.capture('./img')
            cmp += time_rate

            if cmp >= end_time:
                info.save("./img")
                break

        # save and backup
        info.get_info(time.time() - start, error.e_y, error.e_a * 180.0 / math.pi, car.Ax, -car.Vx ** 2 / 0.16 * math.tan(car.u),
                      car.u * 180.0 / math.pi, car.Vx, Vx_des, lane.detected_lane_counts)
        lane.backup()
        stopline.backup()
        car.backup()
        error.backup()

        # dt update
        dt = time.time() - t

    webcam.release()
    cv2.destroyAllWindows()