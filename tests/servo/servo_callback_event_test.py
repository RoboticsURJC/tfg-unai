import RPi.GPIO as GPIO
import datetime



PARALLAX_SERVO_OUTPUT_PIN = 11
PARALLAX_SERVO_FEEDBACK_PIN = 12



def servo_callback(channel):
    if GPIO.input(channel) == GPIO.HIGH:
        print("\n^   at " + str(datetime.datetime.now()))
        print(GPIO.input(PARALLAX_SERVO_FEEDBACK_PIN))
        #why is 0 or 1? how do I propperly read the feedback?
    else:
        print("\n  v at " + str(datetime.datetime.now()))



try:
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(PARALLAX_SERVO_OUTPUT_PIN, GPIO.OUT)
    GPIO.setup(PARALLAX_SERVO_FEEDBACK_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(PARALLAX_SERVO_FEEDBACK_PIN, GPIO.BOTH, callback=servo_callback, bouncetime=200)

    pwm = GPIO.PWM(PARALLAX_SERVO_OUTPUT_PIN, 50)
    pwm.start(0)
    #message =  #like the while True
    while message:=float(input("\nPress Enter to exit.\n")) > 0:
        #pwm.ChangeDutyCycle(message)
        pass


    GPIO.remove_event_detect(PARALLAX_SERVO_FEEDBACK_PIN)

finally:
    GPIO.cleanup()

print("Goodbye!")
