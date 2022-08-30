import math

class Error:
    def __init__(self, alpha_ey = 0.9, alpha_ea = 0.9):
        self.alpha_ea = alpha_ea
        self.alpha_ey = alpha_ey

        self.e_a = 0
        self.e_y = 0

        self.e_a_pre = 0
        self.e_y_pre = 0


    def err_cal(self, left_line, right_line, w):
        if right_line is None and left_line is None:
            self.e_y = self.e_y_pre
            self.e_a = self.e_a_pre
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

            #print("a1: ", a1, "b1: ", b1, "a2: ", a2, "b2: ", b2)
            self.e_a = -math.atan((a1 + a2) / 2)
            self.e_y = -(b1 + b2) / 2 * math.cos(self.e_a)
            self.e_y = self.e_y * self.alpha_ey + self.e_y_pre * (1 - self.alpha_ey)
            self.e_a = self.e_a * self.alpha_ea + self.e_a_pre * (1 - self.alpha_ea)

        self.e_y = self.e_y_pre + min(0.2, max(-0.2, self.e_y - self.e_y_pre))
        self.e_a = self.e_a_pre + min(10 * math.pi / 180, max(-10 * math.pi / 180, self.e_a - self.e_a_pre))

        self.e_y = min(0.4, max(-0.4, self.e_y))
        self.e_a = min(50 * math.pi / 180, max(-50 * math.pi / 180, self.e_a))

        return self.e_y, self.e_a


    def backup(self):
        self.e_a_pre = self.e_a
        self.e_y_pre = self.e_y