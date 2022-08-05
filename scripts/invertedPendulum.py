import serial 
import time 
import numpy as np 
from scipy import signal #module for lowpass filter 
import pandas as pd 

import softParameters
import softFunctions 
import natnetclient as natnet
from natnetclient.natnetclient import tracker


print("Starting NatNet sample", "\n")
print("Instantiating natnet client...", "\n")
client = natnet.NatClient(client_ip = '10.10.17.255', data_port=1511 , comm_port=1510)

arduino = serial.Serial(port='COM4')

myMarker = tracker.Marker()

#write_read takes in a list of strings, where the strings denote the valve readings 
valves = ['','','','','','']


timestamp1 = time.time()


if __name__ == '__main__':

    print("Starting NatNet sample", "\n")
    print("Instantiating natnet class...", "\n")
    softFunctions.write_read(['','','','','',''])
    time.sleep(2)
    softFunctions.write_read(['29','29','','','',''])
    time.sleep(1)
    
timestamp2 = time.time()

print("Runtime Duration: " + str(timestamp2-timestamp1) + "\n")


"""low pas filter section"""

tauLimit = np.pi * 1.2

something = softFunctions.write_read(arduino, valves)

