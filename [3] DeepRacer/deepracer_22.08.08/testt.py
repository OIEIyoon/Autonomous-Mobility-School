from pwm import PWM
import time
import motor

def testt(Vx):
    
    
    pwm0 = PWM(0)
    pwm1 = PWM(1)
    
    pwm0.export()
    pwm1.export()
    pwm0.period = 20000000          # fixed?
    pwm1.period = 20000000
    pwm0.duty_cycle = 1450000       # default value
    pwm1.duty_cycle = 1450000
    pwm0.enable = True
    pwm1.enable = True

    pwm0.duty_cycle = 1405000 - round(60000/5 * Vx)
    pwm1.duty_cycle = 1450000
    time.sleep(1)
    
    pwm0 = PWM(0)
    pwm1 = PWM(1)
    
    pwm0.enable = False
    pwm1.enable = False
    
    pwm0.unexport()
    pwm1.unexport()
    

testt(5.0)

