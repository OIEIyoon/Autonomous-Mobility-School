import cv2
import matplotlib.pyplot as plt
import numpy as np
import os

from Percep.utils.func import Homography, LaneDet


def draw_graph(real_x, real_y, real_x_curv, real_y_curv,
               crop_lineX_lower, crop_lineX_upper, crop_curvX_lower, crop_curvX_upper, rgb):
    xq1 = np.linspace(crop_lineX_lower - 0.5, crop_lineX_upper + 0.5, 50)
    xq2 = np.linspace(crop_curvX_lower - 0.5, crop_curvX_upper + 0.5, 50)

    try:
        poly_coeff_1st = np.polyfit(real_x, real_y, 1)
        yq1 = np.polyval(poly_coeff_1st, xq1)
        plt.scatter(real_x, real_y, color=rgb, s=5)
        plt.plot(xq1, yq1, 'k-', linewidth=2)
    except:
        pass

    try:
        poly_coeff_2nd = np.polyfit(real_x_curv, real_y_curv, 2)
        yq2 = np.polyval(poly_coeff_2nd, xq2)
        poly_coeff_2nd2 = np.polyfit(real_x_curv, real_y_curv, 2)
        yq22 = np.polyval(poly_coeff_2nd2, xq2)
        plt.scatter(real_x_curv, real_y_curv, color=[.7 * _ for _ in rgb], s=.2)
        plt.plot(xq2, yq2, 'k:')
        plt.plot(xq2, yq22, 'm-', linewidth=2)
    except:
        pass



class Camera:
    def __init__(self, H, size = (640, 480), crop_lineX_lower = 0.3, crop_lineX_upper = 0.6,
                 crop_curvX_lower = 0.5, crop_curvX_upper = 1, y1 = 0.3, y2 = 0.4,
                 lower_red = [245, 100, 110], upper_red = [255, 180, 255], lower_green = [0, 200, 180],
                 upper_green = [100, 255, 255], lower_blue = [0, 140, 220], upper_blue = [100, 200, 255],
                 lower_white = [240, 240, 240], upper_white = [255, 255, 255]):
        self.size = size

        self.H = H # homographic view matrix

        self.v_num = 0
        self.VIDEO_PATH = None

        self.ret = False
        self.cap = None
        self.frame = None
        self.c_cnt = 0

        # CROP
        self.crop_lineX_lower = crop_lineX_lower
        self.crop_lineX_upper = crop_lineX_upper
        self.y1 = y1

        self.crop_curvX_lower = crop_curvX_lower
        self.crop_curvX_upper = crop_curvX_upper
        self.y2 = y2

        # RGB range
        self.lower_red = np.array(lower_red, dtype="uint8")
        self.upper_red = np.array(upper_red, dtype="uint8")

        self.lower_green = np.array(lower_green, dtype="uint8")
        self.upper_green = np.array(upper_green, dtype="uint8")

        self.lower_white = np.array(lower_white, dtype="uint8")
        self.upper_white = np.array(upper_white, dtype="uint8")

        self.lower_blue = np.array(lower_blue, dtype="uint8")
        self.upper_blue = np.array(upper_blue, dtype="uint8")


    def get_camera(self, path = '/dev/video'):
        self.VIDEO_PATH = path
        print('\n\n\n')
        print("###################################")
        for self.v_num in range(10):
            try:
                self.cap = cv2.VideoCapture(self.VIDEO_PATH + str(self.v_num))
                if not self.cap.isOpened():
                    print("\n Camera open failed! \n")  # 열리지 않았으면 문자열 출력
                else:
                    print("\n Find CAM \n")
                    break
            except:
                pass
        print("###################################")
        return self.cap


    def get_init_image(self, path = '/img/init'):
        self.read()
        self.imwrite(path, filename = 'init')
        return self.frame


    def get_test_video(self, path):
        try:
            self.cap = cv2.VideoCapture(path)
            if not self.cap.isOpened():
                print("\n Video open failed! \n")  # 열리지 않았으면 문자열 출력
            else:
                print("\n Test Ready \n")
        except:
            pass

        return self.cap


    def read(self):
        try:
            for i in range(2):
                self.ret, self.frame = self.cap.read()
            self.frame = cv2.resize(self.frame, self.size)
        except:
            self.ret = False
            self.frame = None

        return self.ret, self.frame


    def imwrite(self, path, filename = 'None'):
        if not os.path.isdir(path):
            print("No directory, create directory")
            os.makedirs(path)

        try:
            cv2.imwrite(path + '/' + filename + '.png', self.frame)
            print("\n\n\n")
            print("###################################")
            print(filename + ".png is saved!")
            print("###################################")
        except:
            print("\n\n\n")
            print("###################################")
            print("image save fail")
            print("###################################")


    def release(self):
        if self.cap is not None:
            self.cap.release()
            cv2.destroyAllWindows()


    def capture(self, path):
        self.read()
        self.imwrite(path, 'capture_f' + str(self.c_cnt))

        if self.frame is not None:
            img_rgb = self.frame

            plt.grid(True)
            plt.axis("equal")

            # left
            lane, line, curv = LaneDet(img_rgb, self.H, self.lower_red, self.upper_red, self.crop_lineX_lower, self.crop_lineX_upper, self.crop_curvX_lower,
                                     self.crop_curvX_upper)
            draw_graph(line[0], line[1], curv[0], curv[1],
                        self.crop_lineX_lower, self.crop_lineX_upper, self.crop_curvX_lower, self.crop_curvX_upper, [1, 0, 0])

            # right
            lane, line, curv = LaneDet(img_rgb, self.H, self.lower_green, self.upper_green, self.crop_lineX_lower, self.crop_lineX_upper, self.crop_curvX_lower,
                                    self.crop_curvX_upper)
            draw_graph(line[0], line[1], curv[0], curv[1],
                       self.crop_lineX_lower, self.crop_lineX_upper, self.crop_curvX_lower, self.crop_curvX_upper,[0, 1, 0])

            # stop
            lane, line, curv = LaneDet(img_rgb, self.H, self.lower_blue, self.upper_blue, self.crop_lineX_lower, self.crop_lineX_upper, self.crop_curvX_lower,
                                    self.crop_curvX_upper, y1 = self.y1, y2 = 0)
            draw_graph(line[0], line[1], curv[0], curv[1],
                       self.crop_lineX_lower, self.crop_lineX_upper, self.crop_curvX_lower, self.crop_curvX_upper, [0, 0, 1])

            plt.xlim(0, 1.3)
            plt.ylim(-0.5, 0.5)
            
          
            print("\n Capture! \n")
            if not os.path.isdir(path):
                print("No directory, create directory")
                os.makedirs(path)
            plt.savefig(path + "/capture_g" + str(self.c_cnt) + ".png")
            print("###################################")
            print("capture_g" + str(self.c_cnt) + ".png is saved!")
            print("###################################")
            plt.close()
            self.c_cnt += 1
        else:
            print("No Image")




if __name__ == "__main__":
    """
    pointx = [62, 124, 466, 542]
    pointy = [346, 277, 272, 349]
    realx = [0.31, 0.455, 0.455, 0.30]
    realy = [0.15, 0.15, -0.14, -0.14]
    H = Homography(pointx, pointy, realx, realy)


    webcam = Camera(H)
    cap = webcam.get_camera(path = '/dev/video')

    ret, frame = cap.read()
    ret, frame = webcam.read()

    webcam = Camera(H, lower_green = [50, 200, 50], upper_green = [120, 255, 120],
                    lower_red = [200, 50, 50], upper_red = [255, 120, 120],
                    lower_blue = [50, 50, 200], upper_blue = [120, 120, 255])
    webcam.get_test_video("C:/Users/oni/PycharmProjects/deepracer/resource/test_video/test_driving1.mp4")
    webcam.capture('C:/Users/oni/PycharmProjects/deepracer/img')

    webcam.release()
    """
