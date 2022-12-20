import RPi.GPIO as GPIO
import time
import numpy as np
#inPINS = [2,3,4,14,15,18,17,27,22,23]
inPINS = [12]
smoothingWindowLength=4

def getTimex():
    return time.time()

GPIO.setmode(GPIO.BOARD)
GPIO.setup(inPINS, GPIO.IN)
upTimes = [[0] for i in range(len(inPINS))]
downTimes = [[0] for i in range(len(inPINS))]
deltaTimes = [[0] for i in range(len(inPINS))]

def my_callback1(channel):
  i = inPINS.index(channel)
  v = GPIO.input(inPINS[i])
  #GPIO.output(outPINS[0], v) # mirror input state to output state directly (forward servo value only) - don't set PWM then for this pin
  if (v==0):
    downTimes[i].append(getTimex())
    if len(downTimes[i])>smoothingWindowLength: del downTimes[i][0]
  else:
    upTimes[i].append(getTimex())
    if len(upTimes[i])>smoothingWindowLength: del upTimes[i][0]
  deltaTimes[i].append( (downTimes[i][-1]-upTimes[i][-2])/(upTimes[i][-1]-downTimes[i][-1]) )
  if len(deltaTimes[i])>smoothingWindowLength: del deltaTimes[i][0]

GPIO.add_event_detect(inPINS[0], GPIO.BOTH, callback=my_callback1)
#GPIO.add_event_detect(inPINS[1], GPIO.BOTH, callback=my_callback1)

try:
  while True:
    ovl = deltaTimes[0][-smoothingWindowLength:] # output first pin PWM
    ov = sorted(ovl)[len(ovl) // 2] #ov = np.mean(ovl)
    print(ov)
    time.sleep(0.1)
except KeyboardInterrupt:
  GPIO.cleanup()