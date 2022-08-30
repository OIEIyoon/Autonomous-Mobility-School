import numpy as np
import math
import time
import cv2
import os

# Perception
from Percep.Det import Lane, StopLine
from Percep.Camera import Camera

# Planning Control
from Plan.Planning import Error
from Plan.Controller import Controller

# Pwm
from Motor.motor import Motor

# utils
from utils.params import Car, Info
import utils.params as params
from utils.keyPoller import KeyPoller

from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt


def read_rgb_range(filepath="./resource/rgb_range_init.txt"):
    g1 = (0, 200, 180)
    g2 = (100, 255, 255)
    r1 = (235, 100, 110)
    r2 = (255, 180, 255)
    b1 = (0, 140, 230)
    b2 = (100, 200, 255)

    try:
        with open(filepath, "r") as f:
            r = f.readline()
            g = f.readline()
            b = f.readline()
            r = r.split(' ')
            g = g.split(' ')
            b = b.split(' ')

            r1 = [int(r[0]), int(r[1]), int(r[2])]
            r2 = [int(r[3]), int(r[4]), int(r[5])]
            g1 = [int(g[0]), int(g[1]), int(g[2])]
            g2 = [int(g[3]), int(g[4]), int(g[5])]
            b1 = [int(b[0]), int(b[1]), int(b[2])]
            b2 = [int(b[3]), int(b[4]), int(b[5])]
    except:
        pass

    print('\n\n\n')
    print("###################################")
    print("red_lower: ", r1, '\n')
    print("red_upper: ", r2, '\n')
    print("green_lower: ", g1, '\n')
    print("green_upper: ", g2, '\n')
    print("blue_lower: ", b1, '\n')
    print("blue_upper: ", b2, '\n')
    print("###################################")
    return r1, r2, g1, g2, b1, b2


def rgb_message():
    print("\n\n\n")
    print("###################################")
    print("press 'r' for capture red img and get red's rgb range \n")
    print("press 'g' for capture green img and get red's rgb range \n")
    print("press 'b' for capture blue img and get red's rgb range \n")
    print("press 's' for saving rgb range to .txt file \n")
    print("press 'e' for end \n")
    print("###################################")


def init_rgb_range(VIDEO_PATH, IMG_PATH, TXT_PATH):
    print("\n\n\n")
    print("###################################")
    key = KeyPoller()

    with key as poller:
        print("please capture black img. press 'c' for capture black img \n")
        print("###################################")

        while True:
            char = poller.poll()
            if char == 'c':
                webcam = Camera(H=params.H, size=(640, 480))
                webcam.get_camera(path=VIDEO_PATH)
                webcam.read()
                webcam.imwrite(path=IMG_PATH + '/init', filename='black')
                break

    ## make black color table.
    black_table = [[[0 for i in range(256)] for j in range(256)] for k in range(256)]
    black_img = cv2.imread(IMG_PATH + '/init/black.png', cv2.COLOR_BGR2RGB)
    w, h, c = black_img.shape
    for i in range(w):
        for j in range(h):
            r = black_img[i][j][0]
            g = black_img[i][j][1]
            b = black_img[i][j][2]

            black_table[r][g][b] = 1

    # r,g,b range variables
    check = [0, 0, 0]
    reds = []
    greens = []
    blues = []
    g1 = (0, 200, 180)
    g2 = (100, 255, 255)
    r1 = (235, 100, 110)
    r2 = (255, 180, 255)
    b1 = (0, 140, 230)
    b2 = (100, 200, 255)

    # get rgb range
    with key as poller:
        rgb_message()

        while True:
            char = poller.poll()

            if char == 'r':
                reds = []
                print("\n\n\n")
                print("###################################")
                print("capture red img \n")
                webcam.release()
                cv2.destroyAllWindows()

                webcam = Camera(H=params.H, size=(640, 480))
                webcam.get_camera(path=VIDEO_PATH)
                webcam.read()
                webcam.imwrite(path=IMG_PATH + '/init', filename='red')

                # red line rgb list
                red_img = cv2.imread(IMG_PATH + '/init/red.png', cv2.COLOR_BGR2RGB)
                w, h, c = red_img.shape
                for i in range(w):
                    for j in range(h):
                        r = red_img[i][j][0]
                        g = red_img[i][j][1]
                        b = red_img[i][j][2]

                        if black_table[r][g][b] == 0:
                            reds.append([r, g, b])

                # get range
                reds = np.array(reds, dtype=np.uint8)
                r1 = tuple(list(reds.min(axis=0)))
                r2 = tuple(list(reds.max(axis=0)))
                print("\n\n\n")
                print("get red rgb range \n")
                print("###################################")
                check[0] = 1
                rgb_message()

            if char == 'g':
                greens = []
                print("\n\n\n")
                print("###################################")
                print("capture green img \n")
                webcam.release()
                cv2.destroyAllWindows()

                webcam = Camera(H=params.H, size=(640, 480))
                webcam.get_camera(path=VIDEO_PATH)
                webcam.read()
                webcam.imwrite(path=IMG_PATH + '/init', filename='green')

                # green line rgb list
                green_img = cv2.imread(IMG_PATH + '/init/green.png', cv2.COLOR_BGR2RGB)
                w, h, c = green_img.shape
                for i in range(w):
                    for j in range(h):
                        r = green_img[i][j][0]
                        g = green_img[i][j][1]
                        b = green_img[i][j][2]

                        if black_table[r][g][b] == 0:
                            greens.append([r, g, b])

                # get range
                greens = np.array(greens, dtype=np.uint8)
                g1 = tuple(list(greens.min(axis=0)))
                g2 = tuple(list(greens.max(axis=0)))
                print("\n\n\n")
                print("get green rgb range \n")
                print("###################################")
                check[1] = 1
                rgb_message()

            if char == 'b':
                blues = []
                print("\n\n\n")
                print("###################################")
                print("capture blue img \n")
                webcam.release()
                cv2.destroyAllWindows()

                webcam = Camera(H=params.H, size=(640, 480))
                webcam.get_camera(path=VIDEO_PATH)
                webcam.read()
                webcam.imwrite(path=IMG_PATH + '/init', filename='blue')

                # blue line rgb list
                blue_img = cv2.imread(IMG_PATH + '/init/blue.png', cv2.COLOR_BGR2RGB)
                w, h, c = blue_img.shape
                for i in range(w):
                    for j in range(h):
                        r = blue_img[i][j][0]
                        g = blue_img[i][j][1]
                        b = blue_img[i][j][2]

                        if black_table[r][g][b] == 0:
                            blues.append([r, g, b])

                # get range
                blues = np.array(blues, dtype=np.uint8)
                b1 = tuple(list(blues.min(axis=0)))
                b2 = tuple(list(blues.max(axis=0)))
                print("\n\n\n")
                print("get blue rgb range \n")
                print("###################################")
                check[2] = 1
                rgb_message()

            if char == 's':  # save rgb range to txt file.
                print("\n\n\n")
                print("###################################")

                if check[0] == 0:
                    print("not init red range \n")

                if check[1] == 0:
                    print("not init green range \n")

                if check[2] == 0:
                    print("not init blue range \n")

                with open(TXT_PATH + "/rgb_range.txt", "w") as f:
                    try:
                        r = ''
                        r1 = list(r1)
                        for i in range(len(r1)):
                            r = r + str(r1[i]) + ' '
                        r2 = list(r2)
                        for i in range(len(r2)):
                            r = r + str(r2[i]) + ' '
                        f.write(r + '\n')

                        g = ''
                        g1 = list(g1)
                        for i in range(len(g1)):
                            g = g + str(g1[i]) + ' '
                        g2 = list(g2)
                        for i in range(len(g2)):
                            g = g + str(g2[i]) + ' '
                        f.write(g + '\n')

                        b = ''
                        b1 = list(b1)
                        for i in range(len(b1)):
                            b = b + str(b1[i]) + ' '
                        b2 = list(b2)
                        for i in range(len(b2)):
                            b = b + str(b2[i]) + ' '
                        f.write(b + '\n')
                        print("save rgb range at " + TXT_PATH + "/rgb_range.txt \n")
                        print("###################################")
                    except:
                        print("ERROR!")
                        print("###################################")
                        pass
                rgb_message()

            if char == 'e':
                rgb_message()
                print('\n\n\n')
                print("###################################")
                print("red_lower: ", r1, '\n')
                print("red_upper: ", r2, '\n')
                print("green_lower: ", g1, '\n')
                print("green_upper: ", g2, '\n')
                print("blue_lower: ", b1, '\n')
                print("blue_upper: ", b2, '\n')
                print("###################################")
                break

    webcam.release()
    cv2.destroyAllWindows()

    return r1, r2, g1, g2, b1, b2


def help_message():
    print("\n\n\n")
    print("###################################")
    #print("Press 'i' first for calibrating rgb range \n")
    print("Press 'h', if you want to check which functions you can use \n")
    print("Press 'c', capture! \n")
    print("Press 'r', if your camera doesn't work or apply new rbg range \n")
    print("Press 's', make RC car Start. recommend do it after calibrating rgb range, and checking your camera \n")
    print("Press 'e', Stop RC car \n")
    print("Press 'q' if you want to quit this program \n")
    print("###################################")



def get_input():
    """
    print("\n\n\n")
    print("###################################")
    print("Set Lateral Control Gain")
    while True:
        try:
            ky = float(input("lateral distance gain: "))
            break
        except ValueError:
            print("That's not an float! \n")
    print("\n")
    while True:
        try:
            ka = float(input("angle gain: "))
            break
        except ValueError:
            print("That's not an float! \n")
    print("\n\n")

    print("###################################")
    print("Set Longitudinal Control Gain")
    while True:
        try:
            kcv = float(input("lane curvature gain: "))
            break
        except ValueError:
            print("That's not an float! \n")
    print("\n")

    while True:
        try:
            kcl = float(input("clearance gain for stop: "))
            break
        except ValueError:
            print("That's not an float! \n")
    print("\n\n")

    print("###################################")
    print("Set vehicle maximum velocity")
    while True:
        try:
            Vmax = float(input("vehicle velocity max(upper than 0.8): "))
            break
        except ValueError:
            print("That's not an float! \n")
    print("\n\n")

    print("###################################")
    print("Set vehicle maximum lateral acceleration")
    while True:
        try:
            Aymax = float(input("velocity lateral acceleration max: "))
            break
        except ValueError:
            print("That's not an float! \n")
    print("\n\n")
    print("###################################")
    """



    return ky, ka, kcv, kcl, Vmax, Aymax

def read_gain(filepath="./resource/gain_init.txt"):
    ky = 0
    ka= 0
    kcv = 0
    kcl = 0
    Vmax = 0
    Aymax = 0

    try:
        with open(filepath, "r") as f:
            ky = f.readline()
            ka = f.readlline()
            kcv = f.readline()
            kcl = f.readline()
            Vmax = f.readline()
            Aymax = f.readline()
            
            ky = ky.split(' ')
            try:
                ky = float(ky[-1])
            except:
                ky = float(ky[-2])

            ka = ky.split(' ')
            try:
                ka = float(ka[-1])
            except:
                ka = float(ka[-2])

            kcv = kcv.split(' ')
            try:
                kcv = float(kcv[-1])
            except:
                kcv = float(kcv[-2])

            kcl = kcl.split(' ')
            try:
                kcl = float(kcl[-1])
            except:
                kcl = float(kcl[-1])

            Vmax = Vmax.split(' ')
            try:
                Vmax = float(Vmax[-1])
            except:
                Vmax = float(Vmax[-2])

            Aymax = Aymax.split(' ')
            try:
                Aymax = float(Aymax[-1])
            except:
                Aymax = float(Aymax[-1])
    except:
        pass

    print('\n\n\n')
    print("###################################")
    print("ky: ", ky, '\n')
    print("ka: ", ka, '\n')
    print("kcv: ", kcv, '\n')
    print("kcl: ", kcl, '\n')
    print("Vmax: ", Vmax, '\n')
    print("Aymax: ", Aymax, '\n')
    print("###################################")
    return ky, ka, kcv, kcl, Vmax, Aymax



def init(ismotor, left_lower=(0, 200, 180), left_upper=(100, 255, 255),
         right_lower=(235, 100, 110), right_upper=(255, 180, 255), lower_rgb=(0, 140, 230), upper_rgb=(100, 200, 255)):
    ky, ka, kcv, kcl, Vmax, Aymax = get_input()
    # try:
    #    left_lower = tuple(list(np.array(left_lower, dtype = np.uint8)))
    #    left_upper = tuple(list(np.array(left_upper, dtype = np.uint8)))
    #    right_lower = tuple(list(np.array(right_lower, dtype = np.uint8)))
    #    right_upper = tuple(list(np.array(right_upper, dtype = np.uint8)))
    #    lower_rgb = tuple(list(np.array(lower_rgb, dtype = np.uint8)))
    #    upper_rgb = tuple(list(np.array(upper_rgb, dtype = np.uint8)))
    # except:
    #    pass

    try:
        left_lower = np.array(left_lower, dtype=np.uint8)
        left_upper = np.array(left_upper, dtype=np.uint8)
        right_lower = np.array(right_lower, dtype=np.uint8)
        right_upper = np.array(right_upper, dtype=np.uint8)
        lower_rgb = np.array(lower_rgb, dtype=np.uint8)
        upper_rgb = np.array(upper_rgb, dtype=np.uint8)
        # print("\n\n\n YYYYYYYYYYYYYYYYYYY \n\n\n")
    except:
        pass

    car = Car()  # Car state info

    lane = Lane(params.H, w=0.6, left_lower=left_lower, left_upper=left_upper,
                right_lower=right_lower, right_upper=right_upper)  # Detected Lane info
    stopline = StopLine(params.H, pre_clearance=2, alpha_c=0.9,
                        lower_rgb=lower_rgb, upper_rgb=upper_rgb)  # Detected Stopline info

    error = Error(alpha_ey=0.9, alpha_ea=0.9)  # Error module
    controller = Controller(steer_angle_max=30, Vx_max=Vmax, c_min=0.1, tau=1.4, Ay_max=Aymax,
                            k_y=ky, k_a=ka, k_cv=kcv, k_cl=kcl)  # Controller module

    info = Info(size=100000)  # Save driving record

    motor = None
    if ismotor:  # motor flag on
        motor = Motor(vel_max=Vmax, dt=0.01)  # set motor

    dt = 0.1  # time interval

    return [ky, ka, kcv, kcl, Vmax, Aymax], car, lane, stopline, error, controller, info, motor, dt


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
                  car.u * 180.0 / math.pi, car.Vx, Vx_des, lane.detected_lane_counts, lane.max_K)
    lane.backup()
    stopline.backup()
    car.backup()
    error.backup()
