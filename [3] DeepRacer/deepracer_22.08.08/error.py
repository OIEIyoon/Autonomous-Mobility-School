import math
import numpy as np
import matplotlib.pyplot as plt


def update_lane(pre_left_lane_pos, pre_right_lane_pos, left_lane_pos, right_lane_pos, alpha):
    pre_left_lane_pos_x = pre_left_lane_pos[0]
    pre_left_lane_pos_y = pre_left_lane_pos[1]
    pre_right_lane_pos_x = pre_right_lane_pos[0]
    pre_right_lane_pos_y = pre_right_lane_pos[1]

    if type(pre_right_lane_pos_x) == list:
        pre_left_lane_pos_x = np.array(pre_left_lane_pos_x, dtype=np.float)
        pre_left_lane_pos_y = np.array(pre_left_lane_pos_y, dtype=np.float)
        pre_right_lane_pos_x = np.array(pre_right_lane_pos_x, dtype=np.float)
        pre_right_lane_pos_y = np.array(pre_right_lane_pos_y, dtype=np.float)


    left_lane_pos_x = left_lane_pos[0]
    left_lane_pos_y = left_lane_pos[1]
    right_lane_pos_x = right_lane_pos[0]
    right_lane_pos_y = right_lane_pos[1]

    if type(right_lane_pos_x) == list:
        left_lane_pos_x = np.array(left_lane_pos_x, dtype=np.float)
        left_lane_pos_y = np.array(left_lane_pos_y, dtype=np.float)
        right_lane_pos_x = np.array(right_lane_pos_x, dtype=np.float)
        right_lane_pos_y = np.array(right_lane_pos_y, dtype=np.float)


    # nan check
    pre_left_sum = np.sum(np.isnan(pre_left_lane_pos_x).sum()) + np.sum(np.isnan(pre_left_lane_pos_y).sum())
    pre_right_sum = np.sum(np.isnan(pre_right_lane_pos_x).sum()) + np.sum(np.isnan(pre_right_lane_pos_y).sum())
    left_sum = np.sum(np.isnan(left_lane_pos_x).sum()) + np.sum(np.isnan(left_lane_pos_y).sum())
    right_sum = np.sum(np.isnan(right_lane_pos_x).sum()) + np.sum(np.isnan(right_lane_pos_y).sum())


    # update lane if nan elements don't exist.
    if pre_left_sum + pre_right_sum + left_sum + right_sum == 0:
        left_lane_pos_x = alpha * left_lane_pos_x + (1-alpha) * pre_left_lane_pos_x
        left_lane_pos_y = alpha * left_lane_pos_y + (1-alpha) * pre_left_lane_pos_y
        right_lane_pos_x = alpha * right_lane_pos_x + (1-alpha) * pre_right_lane_pos_x
        right_lane_pos_y = alpha * right_lane_pos_y + (1-alpha) * pre_right_lane_pos_y


    return left_lane_pos_x, left_lane_pos_y, right_lane_pos_x, right_lane_pos_y

def err_cal(left_line, right_line, w, e_y_pre, e_a_pre, alpha_ey, alpha_ea):

    if right_line is None and left_line is None:
        e_y = e_y_pre
        e_a = e_a_pre
    else:
        if right_line is not None and left_line is not None:
            a1, b1 = left_line[0], left_line[1]
            a2, b2 = right_line[0], right_line[1]

        elif right_line is None and left_line is not None:
            a1, b1 = left_line[0], left_line[1]
            a2 = a1
            b2 = b1 - w*math.sqrt(1+a2**2)

        elif right_line is not None and left_line is None:
            a2, b2 = right_line[0], right_line[1]
            a1 = a2
            b1 = b2 + w*math.sqrt(1+a1**2)

        e_a = -math.atan((a1+a2)/2)
        e_y = -(b1+b2)/2*math.cos(e_a)
        e_y = e_y * alpha_ey + e_y_pre*(1-alpha_ey)
        e_a = e_a * alpha_ea + e_a_pre*(1-alpha_ea)

    e_y = e_y_pre + min(0.2, max(-0.2, e_y-e_y_pre))
    e_a = e_a_pre + min(10*math.pi/180, max(-10*math.pi/180, e_a-e_a_pre))
    
    e_y = min(0.4, max(-0.4, e_y))
    e_a = min(50*math.pi/180, max(-50*math.pi/180, e_a))

    return e_y, e_a

def max_curv(right_curv, left_curv, max_K_pre, alpha_curv):
    if right_curv is None and left_curv is None:
        max_K = max_K_pre
    else:
        if right_curv is not None and left_curv is not None:
            max_K = right_curv[0] + left_curv[0]
        elif right_curv is None and left_curv is not None:
            max_K = left_curv[0]*2
        elif right_curv is not None and left_curv is None:
            max_K = right_curv[0]*2
        max_K = max_K * alpha_curv + (1-alpha_curv) * max_K_pre
    
    max_K = min(3, max(0, abs(max_K)))

    return max_K

def get_error(route):
    # ego_vehicle pos is (0,0)
    # route's form is y = ax^2 + bx + c
    # (A, B) is the nearest point on y from (0,0) // B = aA^2 + bA + c
    # inner product = 0, so vector1(A,B) and vector2(1, 2aA + b)'s inner product = 0
    # A + 2aAB + bB = A + 2aA(aA^2 + bA + c) + b(aA^2 + bA + c) = 0 's A points are the solutions.
    a = route[0]
    b = route[1]
    c = route[2]

    x = [2 * a**2, 3 * a * b, 2 * a * c + b**2 + 1, b*c]
    ret = np.roots(x)

    # get real roots
    ans = []
    for i in range(len(ret)):
        if type(ret[i]) == np.float64:
            ans.append(ret[i])
        else:  # complex
            if ret[i].imag == 0:
                ans.append(ret[i].real)
    #print("answer: ", ans)

    d = 1e10
    A = 0
    B = 0
    for i in range(len(ans)):
        x = ans[i]
        y = a*x*x + b*x + c

        cmp = math.sqrt(x**2 + y**2)
        if d > cmp:
            A = x
            B = y

    A = A # 최소거리지점 앞으로 살짝 당김
    B = a*A*A + b*A + c
            
    e_y = (A*A + B*B)**(1/2) #distance
    if B > 0:
        e_y = -1 * e_y
    e_a = -1 * math.atan(2*a*A + b) # radian 


    return A, B, e_y, e_a

def curvature(coef, x, y, traj_l):
    # y = coef[0]x^2 + coef[1]x + coef[2]
    # y' = 2 * coef[0]x + coef[1]
    # y'' = 2 * coef[0]

    # k = curvature
    # 곡률이 크면 = R이 작아진다 == 큰 조향각이 필요하다.
    # steer angle -25 ~ 25
    # tan(-25[deg])/0.16 <= k <= tan(25[deg])/0.16
    R_up = math.tan(math.pi * 25 / 180.0) / 0.16
    R_low = math.tan(math.pi * -25 / 180.0) / 0.16

    # position ( A, B) on y 's curvature
    prev_k = abs(2 * coef[0]) / abs((1 + (2 * coef[0] * x + coef[1])**2) ** (3/2))

    if prev_k > R_up: # bound
        prev_k = R_up
    if coef[0] < 0: # direction
        prev_k = -1 * prev_k

    # reference traj's curvature
    x = np.linspace(0, traj_l, 100)
    traj_k = abs(2 * coef[0]) / abs((1 + (2 * coef[0] * x + coef[1])**2) ** (3/2))
    traj_k = traj_k.max()

    if traj_k > R_up: # bound
        traj_k = R_up
    if coef[0] < 0: # direction
        traj_k = -1 * traj_k


    return prev_k, traj_k


