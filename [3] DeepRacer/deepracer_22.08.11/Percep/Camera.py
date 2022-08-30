import cv2
import matplotlib.pyplot as plt
import numpy as np
import os

from Percep.utils.func import Homography

def LaneDet(H, img_rgb, lower, upper, crop_lineX_lower, crop_lineX_upper, crop_curvX_lower, crop_curvX_upper, rgb):
    img_mask = cv2.inRange(img_rgb, lower, upper)
    img_mask = (np.array(img_mask))
    coord = np.where(img_mask >= 1)
    coord = np.array([coord[1], coord[0]])
    ones_vec = np.ones((1, np.size(coord[0])))[0]
    coord = np.insert(coord, 2, [ones_vec], axis=0)

    homo = np.dot(H, coord)
    lane_points = [homo[0] / homo[2], homo[1] / homo[2]]

    idx_x1, idx_x2 = np.where(lane_points[0] > crop_lineX_lower), np.where(lane_points[0] < crop_lineX_upper)
    idx_y1, idx_y2 = np.where(lane_points[1] > -0.3), np.where(lane_points[1] < 0.3)
    idx_x = np.intersect1d(idx_x1, idx_x2)
    idx_y = np.intersect1d(idx_y1, idx_y2)
    idx = np.intersect1d(idx_x, idx_y)
    real_x = lane_points[0][idx]
    real_y = lane_points[1][idx]

    idx_x1_curv, idx_x2_curv = np.where(lane_points[0] > crop_curvX_lower), np.where(lane_points[0] < crop_curvX_upper)
    idx_y1_curv, idx_y2_curv = np.where(lane_points[1] > -0.4), np.where(lane_points[1] < 0.4)
    idx_x_curv = np.intersect1d(idx_x1_curv, idx_x2_curv)
    idx_y_curv = np.intersect1d(idx_y1_curv, idx_y2_curv)
    idx_curv = np.intersect1d(idx_x_curv, idx_y_curv)
    real_x_curv = lane_points[0][idx_curv]
    real_y_curv = lane_points[1][idx_curv]

    xq1 = np.linspace(crop_lineX_lower - 0.5, crop_lineX_upper + 0.5, 50)
    xq2 = np.linspace(crop_curvX_lower - 0.5, crop_curvX_upper + 0.5, 50)

    try:
        # poly_coeff_1st = RANSAC(real_x, real_y, 1)
        poly_coeff_1st = np.polyfit(real_x, real_y, 1)
        yq1 = np.polyval(poly_coeff_1st, xq1)
        if rgb is not None:
            plt.scatter(real_x, real_y, color=rgb, s=5)
            plt.plot(xq1, yq1, 'k-', linewidth=2)
    except Exception as ex:
        poly_coeff_1st = None
        yq1 = None
        pass

    try:
        # poly_coeff_2nd = RANSAC(real_x_curv, real_y_curv, 2)
        poly_coeff_2nd = np.polyfit(real_x_curv, real_y_curv, 2)
        yq2 = np.polyval(poly_coeff_2nd, xq2)
        poly_coeff_2nd2 = np.polyfit(real_x_curv, real_y_curv, 2)
        yq22 = np.polyval(poly_coeff_2nd2, xq2)
        if rgb is not None:
            plt.scatter(real_x_curv, real_y_curv, color=[.7 * _ for _ in rgb], s=.2)
            plt.plot(xq2, yq2, 'k:')
            plt.plot(xq2, yq22, 'm-', linewidth=2)
    except Exception as ex:
        poly_coeff_2nd = None
        yq2 = None
        pass

    print(poly_coeff_1st)
    print(poly_coeff_2nd)
    return img_mask

class Camera:
    def __init__(self, H, size = (640, 480), crop_lineX_lower = 0.3, crop_lineX_upper = 0.6,
                 crop_curvX_lower = 0.5, crop_curvX_upper = 1,
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
        self.crop_curvX_lower = crop_curvX_lower
        self.crop_curvX_upper = crop_curvX_upper

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

        return self.cap


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
            self.ret, self.frame = self.cap.read()
            self.frame = cv2.resize(self.frame, self.size)
        except:
            self.ret = False
            self.frame = None

        return self.ret, self.frame


    def imwrite(self, path):
        if not os.path.isdir(path):
            print("No directory, create directory")
            os.makedirs(path)

        try:
            cv2.imwrite(path + '/capture_f' + str(self.c_cnt) + '.png', self.frame)
        except:
            print("image save fail")


    def release(self):
        if self.cap is not None:
            self.cap.release()


    def capture(self, path):
        self.read()
        self.imwrite(path)

        if self.frame is not None:
            img_color = self.frame
            height, width, _ = img_color.shape
            img_rgb = cv2.cvtColor(img_color, cv2.COLOR_BGR2RGB)
            img_rgb = cv2.resize(img_rgb, self.size)

            plt.grid(True)
            plt.axis("equal")

            img_mask_L = LaneDet(self.H, img_rgb, self.lower_red, self.upper_red, self.crop_lineX_lower, self.crop_lineX_upper, self.crop_curvX_lower,
                                     self.crop_curvX_upper, [1, 0, 0])
            img_mask_R = LaneDet(self.H, img_rgb, self.lower_green, self.upper_green, self.crop_lineX_lower, self.crop_lineX_upper, self.crop_curvX_lower,
                             self.crop_curvX_upper, [0, 1, 0])
            img_mask_S = LaneDet(self.H, img_rgb, self.lower_blue, self.upper_blue, 0, 1, 0, 0, [0, 0, 1])


            plt.xlim(0, 1.3)
            plt.ylim(-0.5, 0.5)

            print("\n Capture! \n")
            if not os.path.isdir(path):
                print("No directory, create directory")
                os.makedirs(path)
            plt.savefig(path + "/capture_g" + str(self.c_cnt) + ".png")
            plt.close()
            self.c_cnt += 1
        else:
            print("No Image")




if __name__ == "__main__":
    pointx = [62, 124, 466, 542]
    pointy = [346, 277, 272, 349]
    realx = [0.31, 0.455, 0.455, 0.30]
    realy = [0.15, 0.15, -0.14, -0.14]
    H = Homography(pointx, pointy, realx, realy)


    webcam = Camera(H)
    cap = webcam.get_camera(path = '/dev/video')

    ret, frame = cap.read()
    ret, frame = webcam.read()

    webcam = Camera(H, lower_green = [90, 200, 90], upper_green = [120, 255, 120],
                    lower_red = [200, 90, 90], upper_red = [255, 120, 120],
                    lower_blue = [90, 90, 200], upper_blue = [120, 120, 255])
    webcam.get_test_video("C:/Users/oni/PycharmProjects/deepracer/resource/test_video/test_driving1.mp4")
    webcam.capture('C:/Users/oni/PycharmProjects/deepracer/img')

    webcam.release()
    cv2.destroyAllWindows()