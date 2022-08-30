import math



def Lateral_control(e_y, e_a, k_y = 4, k_a = 2.5):
    u_lim = 30 * math.pi / 180.0 # radian
    delta = (-k_y * e_y - k_a * e_a) # radian

    return max(-u_lim, min(u_lim, delta))




def Longitudinal_control(Ax_pre, Vx, dt, curv_road, isTarget, clearance, k_cv = 3, k_v = 0.7, k_cl = 0.3):
    Vx_max = 0.4
    c_min = 0.1 # 10cm
    tau = 1.4

    Ay_max = 0.7 # lateral acc constraints
    Vx_des = max(0.7, math.sqrt(min(Vx_max ** 2,  Ay_max / abs(curv_road)))) # using A_y = V_x^2 / R. velocity constraints

    Ax_curv = k_cv * (Vx_des - Vx)

    if isTarget == 1: # stop line detected and use 'clearance control'
        cl_des = c_min + Vx * tau
        Vx_des = 0
        Ax_clrn = -k_cl * (cl_des - clearance) + k_v * (Vx_des - Vx)
        Ax = min([Ax_clrn, Ax_curv, 0]) # must be lower than zero for decreasing speed
    else: # 'velocity control'
        cl_des = None
        Ax = Ax_curv # for Vx -> Vx_des

    #s_max = Vx_max * dt;
    #Ax = Ax_pre + max(-s_max, min(s_max, Ax - Ax_pre));

    return Ax, Vx_des, cl_des
