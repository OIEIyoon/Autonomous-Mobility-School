from pwm import PWM
import time
import motor


def main():
    
    # PWM CONTROL
    motor.pwm_ctrl(0.05,0.1,-20)

try:
    while True:
        if __name__ == "__main__":
            main()

except KeyboardInterrupt:
    
    pwm0 = PWM(0)
    pwm1 = PWM(1)
    
    pwm0.enable = False
    pwm1.enable = False
    
    pwm0.unexport()
    pwm1.unexport()
    
    print('\n\nStopped by Keyboard Interrupt\n')
