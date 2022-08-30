import numpy as np
import math
import time
import cv2

# Perception
from Percep.Det import Lane, StopLine

# Planning Control
from Plan.Planning import Error
from Plan.Controller import Controller

# Pwm
from Motor.motor import Motor

# utils
from utils.params import Car, Info
import utils.params as params



def get_input():
    print("Set Lateral Control Gain")
    while True:
        try:
            ky = float(input("lateral distance gain: "))
            break
        except ValueError:
            print("That's not an float! \n")
    while True:
        try:
            ka = float(input("angle gain: "))
            break
        except ValueError:
            print("That's not an float! \n")
    print("\n\n")

    ###################################
    print("Set Longitudinal Control Gain")
    while True:
        try:
            kcv = float(input("lane curvature gain: "))
            break
        except ValueError:
            print("That's not an float! \n")
    while True:
        try:
            kv = float(input("velocity gain for stop: "))
            break
        except ValueError:
            print("That's not an float! \n")
    while True:
        try:
            kcl = float(input("clearance gain for stop: "))
            break
        except ValueError:
            print("That's not an float! \n")
    print("\n\n")

    ###################################
    print("Set vehicle maximum velocity")
    while True:
        try:
            Vmax = float(input("vehicle velocity max: "))
            break
        except ValueError:
            print("That's not an float! \n")
    print("\n\n")

    ###################################
    print("Set vehicle maximum lateral acceleration")
    while True:
        try:
            Aymax = float(input("velocity lateral acceleration max: "))
            break
        except ValueError:
            print("That's not an float! \n")
    print("\n\n")


    return ky, ka, kcv, kv, kcl, Vmax, Aymax


def init(ismotor):
    ky, ka, kcv, kv, kcl, Vmax, Aymax = get_input()

    car = Car()  # Car state info

    lane = Lane(params.H, w=0.5, left_lower=(0, 200, 180), left_upper=(100, 255, 255),
                right_lower=(235, 100, 110), right_upper=(255, 180, 255))  # Detected Lane info
    stopline = StopLine(params.H, pre_clearance=2, alpha_c=0.9,
                        lower_rgb=(0, 140, 230), upper_rgb=(100, 200, 255))  # Detected Stopline info

    error = Error(alpha_ey=0.9, alpha_ea=0.9)  # Error module
    controller = Controller(steer_angle_max=30, Vx_max=Vmax, c_min=0.1, tau=1.4, Ay_max=Aymax,
                            k_y=ky, k_a=ka, k_cv=kcv, k_v=kv, k_cl=kcl)  # Controller module

    info = Info(size=100000)  # Save driving record

    motor = None
    if ismotor:  # motor flag on
        motor = Motor(vel_max=1.5, dt=0.01)  # set motor

    dt = 0.1  # time interval

    return [ky, ka, kcv, kv, kcl, Vmax, Aymax], car, lane, stopline, error, controller, info, motor, dt


def state_print(start_time, lane, error, car, stopline):
    print("    1/K :      ", round(1 / (lane.max_K + 0.001), 2))
    print("    e_y :      ", round(error.e_y, 2))
    print("    e_a :      ", round(error.e_a * 180 / math.pi, 2))
    print("    Steering : ", round(car.u * 180 / math.pi, 2))
    print("    Vx       : ", round(car.Vx, 2))
    print("    Ax       : ", round(car.Ax, 2))
    if stopline.clearance is not None:
        print("    clearance: ", round(stopline.clearance, 2))
    print("\n\n\n")
    print("running time : ", round(time.time() - start_time, 4))


def backup(start_time, Vx_des, info, lane, stopline, car, error):
    info.get_info(time.time() - start_time, error.e_y, error.e_a * 180.0 / math.pi, car.Ax,
                  -car.Vx ** 2 / 0.16 * math.tan(car.u),
                  car.u * 180.0 / math.pi, car.Vx, Vx_des, lane.detected_lane_counts)
    lane.backup()
    stopline.backup()
    car.backup()
    error.backup()