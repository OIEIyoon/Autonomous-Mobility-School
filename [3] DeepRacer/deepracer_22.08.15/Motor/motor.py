from Motor.pwm import PWM
import time


class Motor:
    def __init__(self, vel_max = 2.0, dt = 0.01):
        self.vel_max = vel_max        # 1.5 m/s
        self.dt = dt            # 1/dt (Hz)

        self.pwm0 = PWM(0)
        print("\n PWM0 is created \n")
        self.pwm1 = PWM(1)
        print("\n PWM1 is created \n")
    	
        # self.pwm0.period = 20000000  # fixed?
        # self.pwm1.period = 20000000


    def stop(self):
        self.pwm0.export()
        self.pwm1.export()

        self.pwm0.duty_cycle = 1450000  # default value
        self.pwm1.duty_cycle = 1450000

        self.pwm0.enable = True
        self.pwm1.enable = True

        self.pwm0.enable = False
        self.pwm1.enable = False

        self.pwm0.unexport()
        self.pwm1.unexport()

        print('\n\nStopped by Keyboard Interrupt\n')


    def kill(self):
        self.stop()
        self.pwm0.enable = False
        self.pwm1.enable = False
        self.pwm0.unexport()
        self.pwm1.unexport()


    def pwm_ctrl(self, a_x, Vx, delta):
        self.pwm0.export()
        self.pwm1.export()

        self.pwm0.period = 20000000  # fixed?
        self.pwm1.period = 20000000

        self.pwm0.duty_cycle = 1450000       # default value
        # pwm1.duty_cycle = 1450000
        self.pwm0.enable = True
        self.pwm1.enable = True

        # Vx = Vx + a_x * dt
        if Vx > self.vel_max:
            Vx = self.vel_max

        self.pwm0.duty_cycle = 1405000 - round(40000.0 * Vx)
        self.pwm1.duty_cycle = 1450000 + round(250000/30 * delta)

        time.sleep(self.dt)

        # print("A_x = %.2f " %a_x, "Vel = %.2f " %vel, "Delta = %.2f " %delta)


    def pwm(self, Vx, delta, dt):
        self.pwm0.export()
        self.pwm1.export()

        self.pwm0.period = 20000000  # fixed?
        self.pwm1.period = 20000000

        self.pwm0.duty_cycle = 1450000       # default value
        # pwm1.duty_cycle = 1450000
        self.pwm0.enable = True
        self.pwm1.enable = True

    #     print("v : ", Vx)
    #     Vx = Vx + a_x * 0.1
    #     if Vx > vel_max
    #         Vx = vel_max

    #     print("Vx : ", Vx)
        self.pwm0.duty_cycle = 1405000 - round(60000 * Vx)
        self.pwm1.duty_cycle = 1450000 + round(250000/30 * delta)

    #     print("a_x = %.2f " %a_x, "v_x = %.2f " %Vx, "Delta = %.2f " %delta)
        time.sleep(dt)
        
        
if __name__ == '__main__':
	Vx = 5
	pwm0 = PWM(0)
	pwm1 = PWM(1)
	
	pwm0.export()
	pwm1.export()
	
	pwm0.period = 20000000
	pwm1.period = 20000000
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
	
	
