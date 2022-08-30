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
from utils.keyPoller import KeyPoller
from utils.params import Car, Info

# PATH
VIDEO = "/dev"
IMAGE = "./img"


webcam = Camera(VIDEO, size = (640, 480))
webcam.get_camera()

key = KeyPoller()


if __name__ == "__main__":
    with key as poller:
        while True:
            char = poller.poll()

            if char == 'q': # program quit
                print("\n Program Quit \n")
                break

            if char == 'c': # capture img
                print("\n Camera Test \n")
                webcam.capture(IMAGE)

            if char == 's': # start driving
                print("\n START !! \n")
                start = time.time()

                # init
                car = Car()

                lane = Lane(w = 0.5 ,left_lower = (0, 200, 180), left_upper = (100, 255, 255),
                 right_lower = (235, 100, 110), right_upper = (255, 180, 255))

                stopline = StopLine(pre_clearance = 2, alpha_c = 0.9, lower_rgb = (0, 140, 230), upper_rgb = (100, 200, 255))

                error = Error(alpha_ey = 0.9, alpha_ea = 0.9)
                controller = Controller(steer_angle_max = 30, Vx_max = 0.4, c_min = 0.1, tau = 1.4, Ay_max = 0.7)

                info = Info(size = 100000)

                motor = None
                ismotor = 1  # motor flag on
                if ismotor:
                    motor = Motor(vel_max=1.5, dt=0.01)

                dt = 0.1

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

                    # planning
                    error.err_cal(lane.left_lane.left_line, lane.right_lane.right_line, w = 0.5)

                    # control
                    car.u = controller.Lateral_control(error.e_y, error.e_a, k_a = 4, k_y = 2.5)
                    car.Ax, Vx_des, cl_des = controller.Longitudinal_control(Ax_pre= car.Ax_pre, Vx= car.Vx, dt = dt, curv_road= lane.max_K,
                                                                         isTarget= stopline.isTarget, clearance= stopline.clearance)
                    car.Vx = car.Vx_pre + car.Ax * dt # dt = 0.1

                    # output
                    if ismotor == 1:
                        motor.pwm_ctrl(0.05, car.Vx, car.u * 180 / math.pi)

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

                    # save and backup
                    info.get_info(time.time() - start, error.e_y, error.e_a * 180.0 / math.pi, car.Ax, -car.Vx ** 2 / 0.16 * math.tan(car.u),
                                  car.u * 180.0 / math.pi, car.Vx, Vx_des, lane.detected_lane_counts)
                    lane.backup()
                    stopline.backup()
                    car.backup()
                    error.backup()

                    # dt update
                    dt = time.time() - t

                    # check end
                    char = poller.poll()
                    if char == 'e': # end driving
                        print("end driving")

                        if ismotor == 1:
                            motor.kill()
                            ismotor = 0

                        info.save(IMAGE)
                        break


webcam.release()
cv2.destroyAllWindows()