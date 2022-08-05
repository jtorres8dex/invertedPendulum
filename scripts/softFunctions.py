import time
import serial
import natnetclient as natnet

from scipy import fft
import numpy as np 


aruduino = serial.Serial(port='COM4', baudrate=115200,timeout=0.1)

def write_read(arduino, valves: list) -> str:
    print("Sending values to serial port...")
    for reading in valves:
        arduino.write(bytes(valves[reading], 'utf-8'))
        print(str(valves.index(reading)), ':', valves[reading])
        time.sleep(0.05)
    data = arduino.readline()
    return data 

"""Lowpass filter: INSERT GITHUB LINK"""





