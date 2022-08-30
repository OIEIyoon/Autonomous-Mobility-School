import cv2 as cv
import numpy as np
import time

hsv = 0
lower_blue1 = 0
upper_blue1 = 0

temp = []

def mouse_callback(event, x, y, flags, param):
    global hsv, lower_blue1, upper_blue1, lower_blue2, upper_blue2, lower_blue3, upper_blue3

    # 마우스 왼쪽 버튼 누를시 위치에 있는 픽셀값을 읽어와서 HSV로 변환합니다.
    if event == cv.EVENT_LBUTTONDOWN:
        print(img_color[y, x])
        color = img_color[y, x]

        one_pixel = np.uint8([[color]])
        #hsv = cv.cvtColor(one_pixel, cv.COLOR_BGR2HSV)
        #hsv = hsv[0][0]
        rgb = cv.cvtColor(one_pixel, cv.COLOR_BGR2RGB)
        rgb = rgb[0][0]
        #print('픽셀의 hsv값',hsv)
        print('픽셀의 rgb값',rgb)
        #temp.append(hsv)
        temp.append(rgb)

        
cv.namedWindow('img_color')
cv.setMouseCallback('img_color', mouse_callback)


frame = cv.imread("home/deepracer/Desktop/img/capture/capture_f10.png")
#frame = cv.imread("/Desktop/33.png")
img_color = frame
    
height, width, _ = img_color.shape
img_color = cv.resize(img_color, (width, height), interpolation=cv.INTER_AREA)
cv.imshow('img_color', img_color)
cv.waitKey()
cv.destroyAllWindows()

#h = []
#s = []
#v = []

r = []
g = []
b = []

for i in range(0,len(temp)):
    r.append(temp[i][0])
    g.append(temp[i][1])
    b.append(temp[i][2])
    #h.append(temp[i][0])
    #s.append(temp[i][1])
    #v.append(temp[i][2])
print("temp",temp)
print("lower",min(r),min(g),min(b))
print("upper",max(r),max(g),max(b))
