#This file is uploaded to the Raspberry Pi device and operforms the signal processing and thresholding for fall detection

import argparse
import serial
import sys
import time
from threading import Thread
from queue import Queue
from time import sleep as sleep
import numpy as np
import copy
import math

# TODO: Make sure teensy_port matches the port for Teensy
# given in the Arduino IDE, and bt_port matches the output
# of the Bluetooth manager when opening a serial connection
# to the HC-05
teensy_port = '/dev/cu.usbmodem110950901'  # Teensy Serial port
bt_port = '/dev/rfcomm0'    # HC-05 port
#bt_baud = 9600
bt_baud = 38400
#CHUNK_SIZE = 1

reset_flag = True
personOK = True

fall_detected_flag = False
start_time = 0

import numpy as np
from twilio.rest import Client

def send_help(phone_number):
	account_sid = "ACe73407588e46a6c2a2510716504f97a8"
	auth_token = "c8a9e7a477a72e931df36a7392159920"
	client = Client(account_sid, auth_token)
	message = client.messages.create(
	  body="Your patient has experienced a fall and is currently motion less",
	  from_="+18449282248",
	  to="+1"+str(phone_number)
	)

def moving_avg(n):
    mem = np.zeros(n)
    avg = 0
    while True:
      x = yield avg/n
      if x is not None:
        old = mem[0]
        mem = np.roll(mem,-1)
        mem[-1] = x
        avg = avg + x - old

def get_data_from_sensor(ser):
  return ser.readline().decode('ascii')

def threshold(ax,ay,az):
    global fall_detected_flag, start_time, personOK
    
    C1 = math.sqrt((ax**2) + (ay**2) + (az**2))         # C1 = sum vector magnitude
    C2 = math.sqrt((ax**2) + (az**2))                   # C2 = sum vector magnitude on horizontal plane
    #C3 = #not sure                                     # C3 = max peak-peak acceleration amplitude
    C4 = math.atan2(math.sqrt((ax**2) + (az**2)), -ay)  # C4 = angle between z axis and vertical
    #print(f'C4:{C4}')
    
    
    C1 = math.exp(C1**2)
    C4 = math.degree(C4) # angle in degree
    #print(f'C1: {C1}\t C4: {C4}')

    if (C1 > 1.2 and C4 > 1):
        print('FALL DETECTED!')
        personOK = False
        fall_detected_flag = True
        start_time = time.time()
        
        

def receive_data(q,ser):    
    while reset_flag:
        #chunk = []
        #while len(chunk) < CHUNK_SIZE:
        #    chunk.append(get_data_from_sensor(ser))
        #q.put(chunk)
        data = get_data_from_sensor(ser)
        q.put(data)
        #print(data)
        
def process_data(q):
    global fall_detected_flag, personOK
    
    N = 4
    ma_x = moving_avg(N)
    next(ma_x)
    ma_y = moving_avg(N)
    next(ma_y)
    ma_z = moving_avg(N)
    next(ma_z)
    while reset_flag:
        data = q.get()
        if data is not None :
            if data[:3] == '***':
                print('Double Tap Detected')
                personOK = True
                fall_detected_flag = False
                
            else:
                data_string = data[:-3]
                data_string_split = data_string.split(',')
                data = np.array(data_string_split).astype(float)
                #print(data)
                ax,ay,az = ma_x.send(data[0]),ma_y.send(data[1]), ma_z.send(data[2])
                threshold(ax,ay,az)
                
                
        if fall_detected_flag:
            time_elapsed = time.time() - start_time
            #print(f'Time elapsed since last fall: {time_elapsed}')
            
            if time_elapsed > 10 and  (not personOK) :
                print(f'Help is on the way!')
                send_help(7348675309) # Sends message for help to phone number specified
                fall_detected_flag = False
                personOK = True
                
            

def imu():
    ''' Measure data rate of bluetooth connection to HC-05 '''
    
    ser = serial.Serial(bt_port, bt_baud)
    
    print('Receiving data...')
    # Sleep for a second for a chance to connect
    time.sleep(1)

    q = Queue()
    # Make daemon threads and start process_data first.
    # When the main thread terminates, the started threads will also terminate:
    p1 = Thread(target=process_data, args=(q,), daemon=True)
    p1.start()
    p2 = Thread(target=receive_data, args=(q,ser,), daemon=True)
    p2.start()
    #
    #print(ser.readline().decode('ascii'))
    
    try:
        input('Hit enter or Ctrl-c to terminate: ')
    except KeyboardInterrupt:
        global reset_flag
        reset_flag = False
        p1.join()
        p2.join()
        pass
    


def main():
    global bt_baud, bt_port, teensy_port
    parser = argparse.ArgumentParser(description='EECS 452 Bluetooth Driver')
    parser.add_argument('function', metavar='function', default='imu',type=str,
                        choices=['imu'],
                        help='The name of the function to run: "imu"')
    parser.add_argument('-b', '--bt-baud', type=int, dest='bt_baud', default=bt_baud,
                        help='Baud Rate for HC-05 Module')
    parser.add_argument('-p', '--bt-port', type=str, dest='bt_port', default=bt_port,
                        help='/dev/<port> for HC-05 Module')
    parser.add_argument('--teensy-port', type=str, dest='teensy_port', default=teensy_port,
                        help='/dev/<port> for Teensy')
    parser.add_argument('-d', '--debug', action='store_true', default=False,
                        help='Print out serial port information for HC-05 and Teensy')
    args = parser.parse_args()

    if args.bt_baud is not None:
        bt_baud = args.bt_baud
    if args.bt_port is not None:
        bt_port = args.bt_port
    if args.teensy_port is not None:
        teensy_port = args.teensy_port

    if args.debug:
        print("Debug:")
        print("bt_baud: ", bt_baud)
        print("bt_port: ", bt_port)
        print("teensy_port: ", teensy_port)
        print()

    if args.function == 'imu':
      imu()

if __name__ == '__main__':
    main()
