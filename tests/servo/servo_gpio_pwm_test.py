import RPi.GPIO as GPIO
import time



PARALLAX_SERVO_OUTPUT_PIN = 11
PARALLAX_SERVO_FEEDBACK_PIN = 12

def setup():
	GPIO.setmode(GPIO.BOARD) #board mode instead of BCM.

	GPIO.setup(PARALLAX_SERVO_OUTPUT_PIN, GPIO.OUT) #GPIO.0 Parallax servo pwm.
	GPIO.setup(PARALLAX_SERVO_FEEDBACK_PIN, GPIO.IN)  #GPIO.1 Parallax servo feedback.
	
	pwm = GPIO.PWM(PARALLAX_SERVO_OUTPUT_PIN, 50)
	pwm.start(0)
	
	return pwm

def clean():
	GPIO.cleanup()

def main():
	pwm = setup()
	
	print(GPIO.input(PARALLAX_SERVO_FEEDBACK_PIN))
	pwm.ChangeDutyCycle(7.5)
	print(GPIO.input(PARALLAX_SERVO_FEEDBACK_PIN))
	time.sleep(1)
	print(GPIO.input(PARALLAX_SERVO_FEEDBACK_PIN))
	pwm.ChangeDutyCycle(5)
	print(GPIO.input(PARALLAX_SERVO_FEEDBACK_PIN))
	time.sleep(1)
	print(GPIO.input(PARALLAX_SERVO_FEEDBACK_PIN))
	pwm.ChangeDutyCycle(10)
	print(GPIO.input(PARALLAX_SERVO_FEEDBACK_PIN))
	time.sleep(1)
	print(GPIO.input(PARALLAX_SERVO_FEEDBACK_PIN))
	
	clean()

if __name__ == "__main__":
	try:
		main()
	except KeyboardInterruption:
		print("Program Interrupted")
		clean()
		
