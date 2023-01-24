import smbus
import time

bus = smbus.SMBus(21)
time.sleep(3)

address = 0x04

def writeNumber(value):
    bus.write_byte(address, int(value))
    #bus.write_byte(address, 0, value
    return -1

def readNumber():
    number = bus.read_byte(address)
    #number = bus.read_byte_data(address, 1)
    return number

while True:
    var = input("Enter 1 - 9: ")
    if not var:
        continue

    writeNumber(var)
    print("RPi: Hi Arduino, I sent you: ", var)
    time.sleep(1)
    number = readNumber()
    print("Arduino: Hey RPi, I received a digit: ", number)
    
