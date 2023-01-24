import smbus
import time

bus = smbus.SMBus(20)
time.sleep(1)

address = 0x04

def writeNumber(value):
    bus.write_byte(address, int(value))
    #bus.write_byte(address, 0, value
    return -1

def readNumber():
    number = bus.read_i2c_block_data(address, 0x02, 4)
    #number = bus.read_byte_data(address, 1)
    return number

while True:

    data = readNumber()
    result = 0
    for b in data:
        result = result * 256 + int(b)
    print(result)
    time.sleep(1)
    