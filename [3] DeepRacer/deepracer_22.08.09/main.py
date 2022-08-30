import cv2
import math
import time
import traceback

# Perception
import Percep.Camera as Camera
import Percep.Det as Det

# Planning Control
import Plan.Planning as Planning
import Plan.Controller as Controller

# Pwm
import Motor.motor as motor
import Motor.stop as stop
from Motor.pwm import PWM

from utils.keyPoller import KeyPoller
from utils.params import Car, Lane, Info, Error



IMAGE = "./img/"


cap = Camera.get_camera()

key = KeyPoller()
c_cnt = 0 # capture image count
r_cnt = 0 # result image count

isTarget = 0 # "stopline detect" flag
ismotor = 0 # "motor on" flag

size = 100000
w = 0.5
dt = 0.1

alpha_ey = 0.9
alpha_ea = 0.9
alpha_curv = 0.1

pre_clearance = 200
clearance = None


if __name__ == "__main__":
    with key as poller:
        while True:
            char = poller.poll()

            if char == 'q': # program quit
                print("\n Program Quit \n")

                stop.stop()
                break

            if char == 'c': # capture img
                print("\n Cameara Test \n")

                ret, frame = cap.read()
                cv2.imwrite('test' + str(c_cnt) + '.png', frame)

                Camera.capture(IMAGE, frame, c_cnt)
                c_cnt += 1

            if char == 's':
                print("\n START !! \n")
                start = time.time()

                while True:
                    detected_lane_counts = 0
                    ret, frame = cap.read()

                    #init
                    car = Car()
                    lane = Lane(w)
                    info = Info(size)
                    error = Error()

                    if (ret):
                        #####################
                        ##### RIGHT LANE #####
                        #####################
                        try:
                            right_lane = Det.LaneDet(frame, (235, 100, 110), (255, 180, 255))
                            lane.right_line = right_lane[0]
                        except Exception as ex:
                            right_line = None
                            pass
                        try:
                            lane.right_curv = right_lane[1]
                        except:
                            lane.right_curv = None
                            pass

                        ######################
                        ##### LEFT LANE #####
                        ######################
                        try:
                            left_lane = Det.LaneDet(frame, (0, 200, 180), (100, 255, 255))
                            lane.left_line = left_lane[0]
                        except Exception as ex:
                            lane.left_line = None
                            pass
                        try:
                            lane.left_curv = left_lane[1]
                        except:
                            lane.left_curv = None
                            pass

                        #####################
                        ##### Stop Line #####
                        #####################
                        try:
                            stop_line = Det.StopLineDet(frame, (0, 140, 230), (100, 200, 255))
                            isTarget = 1
                            a = stop_line[0]
                            b = stop_line[1]
                            clearance = min(abs(1 / math.sqrt(1 + a ** 2)), pre_clearance)
                        except:
                            isTarget = 0
                            clearance = None

                        if lane.left_line is not None:
                            detected_lane_counts = detected_lane_counts + 1
                        if lane.right_line is not None:
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
                    error.e_y, error.e_a = Planning.err_cal(lane.left_line, lane.right_line, w, error.e_y_pre, error.e_a_pre, alpha_ey, alpha_ea)
                    lane.max_K = Planning.max_curv(lane.right_curv, lane.left_curv, lane.max_K_pre, alpha_curv)



                    ###################
                    ##### CONTROL #####
                    ###################
                    u = Controller.Lateral_control(error.e_y, error.e_a)
                    car.Ax, Vx_des, cl_des = Controller.Longitudinal_control(Ax_pre=car.Ax_pre, Vx=car.Vx, dt=dt, curv_road=lane.max_K,
                                                                         isTarget=isTarget, clearance=clearance)
                    car.Vx = car.Vx_pre + car.Ax * dt  # option
                    t = time.time()  # loop period

                    ###############
                    ##### PWM #####
                    ###############
                    if ismotor == 1:
                        motor.pwm_ctrl(0.05, car.Vx, u * 180 / math.pi)


                    #################
                    ##### PRINT #####
                    #################
                    print("    1/K :      ", round(1 / (lane.max_K + 0.001), 2))
                    print("    e_y :      ", round(error.e_y, 2))
                    print("    e_a :      ", round(error.e_a * 180 / math.pi, 2))
                    print("    Steering : ", round(u * 180 / math.pi, 2))
                    print("    Vx       : ", round(car.Vx, 2))
                    print("    Ax       : ", round(car.Ax, 2))
                    if clearance is not None:
                        print("    clearance: ", round(clearance, 2))
                    print("\n\n\n")
                    print("running time : ", round(time.time() - start, 4))


                    ################
                    ##### SAVE #####
                    ################
                    info.get_info(time.time() - start, error.e_y, error.e_a * 180.0 / math.pi, car.Ax, -car.Vx ** 2 / 0.16 * math.tan(u),
                                  u * 180.0 / math.pi, car.Vx, Vx_des, detected_lane_counts)
                    lane.backup()
                    car.backup()
                    error.backup()

                    if clearance is not None:
                        pre_clearance = clearance

                    # check end
                    char = poller.poll()
                    if char == 'e': # end driving
                        print("end")

                        if ismotor == 1:
                            pwm0 = PWM(0)
                            pwm1 = PWM(1)
                            pwm0.enable = False
                            pwm1.enable = False
                            pwm0.unexport()
                            pwm1.unexport()

                        info.save(IMAGE, r_cnt)
                        r_cnt += 1
                        break


cap.release()
cv2.destroyAllWindows()