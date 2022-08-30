import math
import numpy as np
import matplotlib.pyplot as plt


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
            b2 = b1 - w * math.sqrt(1 + a2 ** 2)

        elif right_line is not None and left_line is None:
            a2, b2 = right_line[0], right_line[1]
            a1 = a2
            b1 = b2 + w * math.sqrt(1 + a1 ** 2)

        e_a = -math.atan((a1 + a2) / 2)
        e_y = -(b1 + b2) / 2 * math.cos(e_a)
        e_y = e_y * alpha_ey + e_y_pre * (1 - alpha_ey)
        e_a = e_a * alpha_ea + e_a_pre * (1 - alpha_ea)

    e_y = e_y_pre + min(0.2, max(-0.2, e_y - e_y_pre))
    e_a = e_a_pre + min(10 * math.pi / 180, max(-10 * math.pi / 180, e_a - e_a_pre))

    e_y = min(0.4, max(-0.4, e_y))
    e_a = min(50 * math.pi / 180, max(-50 * math.pi / 180, e_a))

    return e_y, e_a


def max_curv(right_curv, left_curv, max_K_pre, alpha_curv):
    if right_curv is None and left_curv is None:
        max_K = max_K_pre
    else:
        if right_curv is not None and left_curv is not None:
            max_K = right_curv[0] + left_curv[0]
        elif right_curv is None and left_curv is not None:
            max_K = left_curv[0] * 2
        elif right_curv is not None and left_curv is None:
            max_K = right_curv[0] * 2
        max_K = max_K * alpha_curv + (1 - alpha_curv) * max_K_pre

    max_K = min(3, max(0, abs(max_K)))

    return max_K
