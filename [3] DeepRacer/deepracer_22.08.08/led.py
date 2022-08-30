from pwm import PWM
import time

def led_on():
	pwm2 = PWM(2)
	pwm3 = PWM(3)
	pwm4 = PWM(4)

	pwm2.export()
	pwm3.export()
	pwm4.export()
	
	pwm2.period = 20000000
	pwm3.period = 20000000
	pwm4.period = 20000000
		
	pwm2.duty_cycle = 1450000 
	pwm2.enable = True
	
	pwm3.duty_cycle = 1450000 
	pwm3.enable = True
	
	pwm4.duty_cycle = 1450000 
	pwm4.enable = True	
	
	
def led_off():
	
	pwm2 = PWM(2)
	pwm3 = PWM(3)
	pwm4 = PWM(4)

	pwm2.export()
	pwm3.export()
	pwm4.export()
	
	pwm2.enable = False
	pwm2.unexport()

	pwm3.enable = False
	pwm3.unexport()


	pwm4.enable = False
	pwm4.unexport()

led_ctrl()


