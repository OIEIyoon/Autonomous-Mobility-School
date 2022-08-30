from pwm import PWM
import time

# Global Variables
# vel = 0
vel_max = 1.5        # 1.5 m/s
dt = 0.01            # 1/dt (Hz)

pwm0 = PWM(0)
pwm1 = PWM(1)

def pwm_ctrl(a_x, Vx, delta):
    
    global pwm0, pwm1

    pwm0.export()
    pwm1.export()
    pwm0.period = 20000000          # fixed?
    pwm1.period = 20000000
    pwm0.duty_cycle = 1450000       # default value
    # pwm1.duty_cycle = 1450000
    pwm0.enable = True
    pwm1.enable = True

#     Vx = Vx + a_x * dt
    if Vx > vel_max :
        Vx = vel_max

    pwm0.duty_cycle = 1405000 - round(40000.0 * Vx)
    pwm1.duty_cycle = 1450000 + round(250000/30 * delta)
    time.sleep(dt)

    # print("A_x = %.2f " %a_x, "Vel = %.2f " %vel, "Delta = %.2f " %delta)

    
def pwm(Vx, delta, dt):
    
    global pwm0, pwm1

    pwm0.export()
    pwm1.export()
    pwm0.period = 20000000          # fixed?
    pwm1.period = 20000000
    pwm0.duty_cycle = 1450000       # default value
    # pwm1.duty_cycle = 1450000
    pwm0.enable = True
    pwm1.enable = True
    
#     print("v : ", Vx)
#     Vx = Vx + a_x * 0.1
#     if Vx > vel_max
#         Vx = vel_max

#     print("Vx : ", Vx)        
        
    pwm0.duty_cycle = 1405000 - round(60000 * Vx)
    pwm1.duty_cycle = 1450000 + round(250000/30 * delta)
    
#     print("a_x = %.2f " %a_x, "v_x = %.2f " %Vx, "Delta = %.2f " %delta)
    
    time.sleep(dt)