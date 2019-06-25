import threading
import time

import serial

ser = serial.Serial('COM3', baudrate=9600, timeout=1)


def getValues():
    ser.write(b'g')
    arduinoData = ser.readline().decode()

    return arduinoData


datalist = []

print('hi')

while (True):

    i = input  # this is where the user either choses to input "Enter"
    # or to let the loop continue
    if i:
        break

no_input = True


def add_up_time():
    print('starting ... ')
    while no_input:
        data = getValues()
        datalist.append(data)
    with open(time.strftime("%Y-%m-%d %H-%M-%S", time.gmtime()) + '.csv', 'w') as file:
        for e in datalist:
            file.write(e[:-1])


# designed to be called as a thread
def signal_user_input():
    global no_input
    i = input("hit enter to stop things")  # I have python 2.7, not 3.x
    no_input = False
    # thread exits here


# we're just going to wait for user input while adding up time once...
threading.Thread(target=signal_user_input).start()

add_up_time()
