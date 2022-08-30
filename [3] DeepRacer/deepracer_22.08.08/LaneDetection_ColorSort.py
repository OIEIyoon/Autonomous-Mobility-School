import cv2
import numpy as np
import matplotlib.pyplot as plt

frame = cv2.imread('Image/Color_Track1.png')
#frame = cv2.VideoCapture("/dev/cam0")

#lower_blue = (110, 30, 30) # hsv 이미지에서 바이너리 이미지로 생성 , 적당한 값 30
#upper_blue = (130, 255, 255)
#img_mask = cv2.inRange(hsv, lower_blue, upper_blue) # 범위내의 픽셀들은 흰색, 나머지 검은색

## 바이너리 이미지를 마스크로 사용하여 원본이미지에서 범위값에 해당하는 영상부분을 획득
#img_result = cv2.bitwise_and(frame,frame, mask = img_mask) 


#plt.imshow(img_result)
#plt.show()


#lower_red = (0, 50, 50) # hsv 이미지에서 바이너리 이미지로 생성 , 적당한 값 30
#upper_red = (10, 255, 255)
#img_mask = cv2.inRange(hsv, lower_red, upper_red) # 범위내의 픽셀들은 흰색, 나머지 검은색

### 바이너리 이미지를 마스크로 사용하여 원본이미지에서 범위값에 해당하는 영상부분을 획득
#img_result = cv2.bitwise_and(frame,frame, mask = img_mask) 


#plt.imshow(img_result)
#plt.show()

def detect_blue(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blur = cv2.GaussianBlur(frame, (5, 5), 0)
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    # filter for blue lane lines
    lower_blue = np.array([90, 190, 100], dtype = "uint8")
    upper_blue = np.array([110, 255, 230], dtype = "uint8")
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_white = cv2.inRange(gray, 200, 255)
    mask_bu = cv2.bitwise_or(mask_white, mask_blue)
    mask_bu_image = cv2.bitwise_and(gray, mask_bu)

    # detect edges
    edges = cv2.Canny(mask_bu_image, 200, 400)

    return edges

#img_result = detect_blue(frame)
#plt.imshow(img_result, cmap='gray')
#plt.show()

def detect_red(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blur = cv2.GaussianBlur(frame, (5, 5), 0)
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    # filter for red lane lines
    lower_red = np.array([170, 100, 200], dtype = "uint8")
    upper_red = np.array([180, 135, 235], dtype = "uint8")
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    mask_white = cv2.inRange(gray, 200, 255)
    mask_re = cv2.bitwise_or(mask_white, mask_red)
    mask_re_image = cv2.bitwise_and(gray, mask_re)

    # detect edges
    edges = cv2.Canny(mask_re_image, 200, 400)

    return edges

#img_result = detect_red(frame)
#plt.imshow(img_result, cmap='gray')
#plt.show()

def region_of_interest(edges):
    height, width = edges.shape
    mask = np.zeros_like(edges)

    # only focus bottom half of the screen
    polygon = np.array([[
        (0, height * 1 / 2),
        (width, height * 1 / 2),
        (width, height * 2 / 3),
        (0, height * 2 / 3),
    ]], np.int32) # 화면에서 잘라낼 부분 삼각형 각각 꼭짓점 좌표(나중에 화면 보고 수정)

    cv2.fillPoly(mask, polygon, 255)
    cropped_edges = cv2.bitwise_and(edges, mask)

    return cropped_edges

def detect_line_segments(cropped_edges):
    # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
    rho = 1  # distance precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
    min_threshold = 10  # minimal of votes
    line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, 
                                    np.array([]), minLineLength=8, maxLineGap=4)

    return line_segments

img_result = region_of_interest(detect_blue(frame))
plt.imshow(img_result, cmap='gray')
plt.show()
print(detect_line_segments(img_result))

def average_slope_intercept(frame, line_segments):
    """
    This function combines line segments into one or two lane lines
    If all line slopes are < 0: then we only have detected left lane
    If all line slopes are > 0: then we only have detected right lane
    """
    lane_lines = []
    if line_segments is None:
        logging.info('No line_segment segments detected')
        return lane_lines

    height, width= frame.shape
    lane_fit = []

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            lane_fit.append((slope, intercept))
            

    lane_fit_average = np.average(lane_fit, axis=0)
    if len(lane_fit) > 0:
        lane_lines.append(make_points(frame, lane_fit_average))

    return lane_lines
#def average_slope_intercept(frame, line_segments):
#    for line_segment in line_segments:
#        for x1, y1, x2, y2 in line_segment:


def make_points(frame, line):
    height, width = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

    # bound the coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]

print(average_slope_intercept(img_result, detect_line_segments(img_result)))

def Homography():
    # 바닥을 기준으로 한 point x/y 는 image pixel 상 2차원 좌표, real x/y 는 실제 트랙 위 3차원 좌표(z=0) 4개를 넣어주면 호모그래피 행렬 H 값 나옴.
    pointx = [368,52,330,162]
    pointy = [317,269,184,174]
    realx = [46,51,76,80]
    realy = [7,-8,9,-11]

    pts_src = np.array([[pointx[0], pointy[0]],[pointx[1], pointy[1]],[pointx[2], pointy[2]],[pointx[3], pointy[3]]])
    pts_dst = np.array([[realx[0], realy[0]],[realx[1], realy[1]],[realx[2], realy[2]],[realx[3], realy[3]]])
    # Calculate Homography
    h, status = cv2.findHomography(pts_src, pts_dst)
    return h

def frame_points(points):
    a = np.polyfit((points[0][0][0],points[0][0][2]),(points[0][0][1],points[0][0][3]),1)
    b = np.linspace(points[0][0][0],points[0][0][2],20)
    point = []
    real = []
    realx = []
    realy = []
    H = Homography()
    lane_points = []
    i = 0
    for x in b:
        i = a[0] * x + a[1]
        point.append([x,i,1])
    for y in range(20):
        real.append(H@(np.transpose(point[y])))
    for z in range(20):
        realx.append((np.transpose(real[z]))[0]/(np.transpose(real[z]))[2])
        realy.append((np.transpose(real[z]))[1]/(np.transpose(real[z]))[2])
    #for z in range(20):
    lane_points = [realx, realy]
    return lane_points

print(frame_points(average_slope_intercept(img_result, detect_line_segments(img_result))))
print(frame_points(average_slope_intercept(img_result, detect_line_segments(img_result)))[0])
\

def display_lines(frame, lines, line_color=(0, 255, 0), line_width=2):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image

lane_lines = average_slope_intercept(img_result, detect_line_segments(img_result))
lane_lines_image = display_lines(frame, lane_lines)
plt.imshow(lane_lines_image)
plt.show()

