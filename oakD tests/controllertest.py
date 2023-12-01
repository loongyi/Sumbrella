#controller test
import sys
import numpy as np
import time
import serial

controller = serial.Serial(port='COM6',baudrate=115200, timeout=1)
def write_control(cccc):
    controller.write(bytes(cccc, 'utf-8'))
    time.sleep(0.05)
    data = 'Serial sent: ' + str(cccc)
    print(data)
    return data

COMMAND =0

while True:
    COMMAND = 0
    data = controller.readline().rstrip()
    if data:
        COMMAND = int(float(data))
        print (COMMAND)
    if COMMAND == 10:
        write_control('5\n')
    if COMMAND == 11:
        write_control('6\n')