#!/usr/bin/env python3
import serial, time

if __name__ == "__main__":
    #serial_arduino = serial.Serial("/dev/ttyACM0", 115200, timeout=1) #via USB.
    serial_arduino = serial.Serial("/dev/ttyAMA0", 115200, timeout=1) #via RX and TX pins.
    serial_arduino.reset_input_buffer()

    try:
        while True:
            start_time = time.time() * 1000 #millis
            if serial_arduino.in_waiting > 0:
                msg = serial_arduino.readline().decode("utf8", "ignore").rstrip()
                end_time = time.time() * 1000 #millis
                print(f"[{end_time - start_time:1.4f}]\t{msg}")
    except KeyboardInterrupt:
        serial_arduino.close()
