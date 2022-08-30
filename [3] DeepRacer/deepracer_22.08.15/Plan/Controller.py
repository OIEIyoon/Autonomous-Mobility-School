import math



class Controller:
    def __init__(self, steer_angle_max = 30, Vx_max = 0.4, c_min = 0.1, tau = 1.4, Ay_max = 0.7,
                 k_y = 4, k_a = 2.5, k_cv = 3, k_cl = 0.3):
        self.steer_angle_max = steer_angle_max
        self.c_min = c_min
        self.tau = tau

        # lateral control gain
        self.k_y = k_y
        self.k_a = k_a

        # longitudinal control gain
        self.k_cv = k_cv
        #self.k_v = k_v
        self.k_cl = k_cl
        self.Vx_max = Vx_max
        self.Ay_max = Ay_max

        self.ee_y = 0
        self.ee_a = 0


    def Lateral_control(self, e_y, e_a, dt):
        #self.ee_y += e_y * dt
        #self.ee_a += e_a * dt
        u_lim = self.steer_angle_max * math.pi / 180.0 # radian
        
        #I_delta = max(-4, min(4,  -0.1 * (self.ee_y + self.ee_a)))#radian
        delta = (-self.k_y * e_y - self.k_a * e_a)# + I_delta #radian
        

        return max(-u_lim, min(u_lim, delta))




    def Longitudinal_Control(self, Ax_pre, Vx, dt, curv_road, isTarget, clearance, BSC_pre, ax_nom_pre):
        # Vx: 현재 속도
        ax_set = -0.5
        Init = 0
        curvature = abs(curv_road) + 0.0001

        # 곡률기반 속도 제어
        Vx_des = max(1.2, math.sqrt(min(self.Vx_max ** 2, self.Ay_max / (
                                            curvature))))  # using A_y = V_x^2 / R. velocity constraints
        a1 = self.k_cv * (Vx_des - Vx)

        # STOP
        if isTarget == 1:
            if BSC_pre == 1:
                BSC = 1
            else:
                if (Vx ** 2 / (-2 * ax_set) + self.c_min > clearance):
                    Init = 1
                    BSC = 1
                else:
                    BSC = 0
        else:
            BSC = 0


        # normalize x acc
        if Init == 1:
            ax_nom = -Vx ** 2 / 2 / (clearance - self.c_min)
        else:
            ax_nom = ax_nom_pre

        #
        if BSC == 1:
            # gain: k4
            d_des = -Vx ** 2 / (2 * ax_nom) + self.c_min
            a2 = ax_nom + self.k_cl * (d_des - clearance)
            Ax = min([a1, a2, 0])
            Vx_des = 0
        else:
            d_des = None
            Ax = a1


        # s_max = 2 * dt;
        # Ax = Ax_pre + max(-s_max, min(s_max, Ax - Ax_pre));
        return Ax, Vx_des, d_des, BSC, ax_nom
