import pigpio
import time

pi1 = pigpio.pi()

pi1.set_mode(11, pigpio.OUTPUT)
pi1.set_mode(12, pigpio.INPUT)



pi1.set_PWM_dutycycle(11, 75)
pi1.set_pull_up_down(12, pigpio.PUD_UP)

time.sleep(1)
while True:
    #print(pi1.get_PWM_dutycycle(11))
    print(pi1.read(12))
