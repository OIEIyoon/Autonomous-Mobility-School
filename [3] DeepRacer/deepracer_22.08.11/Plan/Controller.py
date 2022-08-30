import math



class Controller:
    def __init__(self, steer_angle_max = 30, Vx_max = 0.4, c_min = 0.1, tau = 1.4, Ay_max = 0.7,
                 k_y = 4, k_a = 2.5,  k_cv = 3, k_v = 0.7, k_cl = 0.3):
        self.steer_angle_max = steer_angle_max
        self.c_min = c_min
        self.tau = tau

        # lateral control gain
        self.k_y = k_y
        self.k_a = k_a

        # longitudinal control gain
        self.k_cv = k_cv
        self.k_v = k_v
        self.k_cl = k_cl
        self.Vx_max = Vx_max
        self.Ay_max = Ay_max

    def Lateral_control(self, e_y, e_a):
        u_lim = self.steer_angle_max * math.pi / 180.0 # radian
        delta = (-self.k_y * e_y - self.k_a * e_a) # radian

        return max(-u_lim, min(u_lim, delta))


    def Longitudinal_control(self, Ax_pre, Vx, dt, curv_road, isTarget, clearance):
        Vx_des = max(0.7, math.sqrt(min(self.Vx_max ** 2,  self.Ay_max / abs(curv_road + 0.001)))) # using A_y = V_x^2 / R. velocity constraints

        Ax_curv = self.k_cv * (Vx_des - Vx)

        if isTarget == 1: # stop line detected and use 'clearance control'
            cl_des = self.c_min + Vx * self.tau
            Vx_des = 0
            Ax_clrn = -self.k_cl * (cl_des - clearance) + self.k_v * (Vx_des - Vx)
            Ax = min([Ax_clrn, Ax_curv, 0]) # must be lower than zero for decreasing speed
        else: # 'velocity control'
            cl_des = None
            Ax = Ax_curv # for Vx -> Vx_des

        #s_max = Vx_max * dt;
        #Ax = Ax_pre + max(-s_max, min(s_max, Ax - Ax_pre));

        return Ax, Vx_des, cl_des
