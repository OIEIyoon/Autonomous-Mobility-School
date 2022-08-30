import cv2

# Perception
from Percep.Camera import Camera

# utils
from utils.keyPoller import KeyPoller


def rgb_range_init(VIDEO_PATH):
    key = KeyPoller()
    with key as poller:
        print("please capture black img. press 'c' for capture black img \n")

        while True:
            char = poller.poll()
            if char == 'c':
                webcam = Camera(H=params.H, size=(640, 480))
                webcam.get_camera(path=VIDEO_PATH)
                webcam.read()
                webcam.imwrite(path='/img/init', filename='black')
                break


    ## make black color table.
    black_table = [[[0 for i in range(256)] for j in range(256)] for k in range(256)]
    black_img = cv2.imread('/img/init/black.png', cv2.COLOR_BGR2RGB)
    w, h, c = black_img.shape
    for i in range(w):
        for j in range(h):
            r = black_img[w][h][0]
            g = black_img[w][h][1]
            b = black_img[w][h][2]

            black_table[r][g][b] = 1

    # r,g,b range variables
    check[3] = [0, 0, 0]
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
        print("press 'r' for capture red img and get red's rgb range \n")
        print("press 'g' for capture green img and get red's rgb range \n")
        print("press 'b' for capture blue img and get red's rgb range \n")
        print("press 's' for saving rgb range to .txt file \n")
        print("press 'e' for end \n")

        while True:
            char = poller.poll()

            if char == 'r':
                print("capture red img \n")
                webcam.release()
                webcam = Camera(H=params.H, size=(640, 480))
                webcam.get_camera(path=VIDEO_PATH)
                webcam.read()
                webcam.imwrite(path = '/img/init', filename='red')

                # red line rgb list
                red_img = cv2.imread('/img/init/red.png', cv2.COLOR_BGR2RGB)
                w, h, c = red_img.shape
                for i in range(w):
                    for j in range(h):
                        r = red_img[w][h][0]
                        g = red_img[w][h][1]
                        b = red_img[w][h][2]

                        if black_table[r][g][b] == 0:
                            reds.append([r,g,b])

                # get range
                red_img = np.array(red_img, dtype = np.uint32)
                r1 = tuple(list(red_img.min(axis = 1)))
                r2 = tuple(list(red_img.max(axis = 1)))
                print("get red rgb range \n")
                check[0] = 1


            if char == 'g':
                print("capture green img \n")
                webcam.release()
                webcam = Camera(H=params.H, size=(640, 480))
                webcam.get_camera(path=VIDEO_PATH)
                webcam.read()
                webcam.imwrite(path = '/img/init', filename='green')

                # green line rgb list
                green_img = cv2.imread('/img/init/green.png', cv2.COLOR_BGR2RGB)
                w, h, c = green_img.shape
                for i in range(w):
                    for j in range(h):
                        r = green_img[w][h][0]
                        g = green_img[w][h][1]
                        b = green_img[w][h][2]

                        if black_table[r][g][b] == 0:
                            greens.append([r, g, b])

                # get range
                green_img = np.array(green_img, dtype=np.uint32)
                g1 = tuple(list(green_img.min(axis=1)))
                g2 = tuple(list(green_img.max(axis=1)))
                print("get green rgb range \n")
                check[1] = 1


            if char == 'b':
                print("capture blue img \n")
                webcam.release()
                webcam = Camera(H=params.H, size=(640, 480))
                webcam.get_camera(path=VIDEO_PATH)
                webcam.read()
                webcam.imwrite(path = '/img/init', filename='blue')

                # blue line rgb list
                blue_img = cv2.imread('/img/init/blue.png', cv2.COLOR_BGR2RGB)
                w, h, c = blue_img.shape
                for i in range(w):
                    for j in range(h):
                        r = blue_img[w][h][0]
                        g = blue_img[w][h][1]
                        b = blue_img[w][h][2]

                        if black_table[r][g][b] == 0:
                            blues.append([r, g, b])

                # get range
                blue_img = np.array(blue_img, dtype=np.uint32)
                b1 = tuple(list(blue_img.min(axis=1)))
                b2 = tuple(list(blue_img.max(axis=1)))
                print("get blue rgb range \n")
                check[2] = 1


            if char == 's': # save rgb range to txt file.
                if check[0] == 0:
                    print("not init red range \n")

                if check[1] == 0:
                    print("not init green range \n")

                if check[2] == 0:
                    print("not init blue range \n")

                file = open("/resource/rgb_range.txt", "w")
                file.write(r1, '\n')
                file.write(r2, '\n')
                file.write(g1, '\n')
                file.write(g2, '\n')
                file.write(b1, '\n')
                file.write(b2, '\n')
                print("save rgb range at '/resource/rgb_range.txt'\n")

            if char == 'e':
                break

    webcam.release()
    cv2.destroyAllWindows()

    return r1,r2,g1,g2,b1,b2