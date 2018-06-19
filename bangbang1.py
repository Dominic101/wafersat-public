# -*- coding: utf-8 -*-
"""
Created on Mon Jun 18 10:38:42 2018

@author: podsa

Bang-Bang Controller 
Simple thermal controller

Records the temperature of the 8 RTD sensors. Based on the average temperature of 
sensors, the algorithm turns the heaters on or off (50% duty cycle)
"""

import time
import RPi.GPIO as GPIO
import csv
import serial
from numpy import mean
from math import pi, sin, radians
import sh
from time import sleep
import subprocess
from datetime import datetime
import newRTD_V2 as rtd 
import os


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(25,GPIO.OUT)
heater = GPIO.PWM(25,20)
heater.start(0)
# filename formatting
time = datetime.now()
time = time.strftime("%Y-%m-%d %H:%M")
filename = 'pi_stats' + time + '.csv'
filename = filename.replace(' ', '')
print('Filename for generated file will be:')
print(filename)

#ser = serial.Serial('/dev/ttyUSB0', 19200, timeout = 1)

#print(rtd.RDT_list,'!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')

def TFD(data): 
    """
    For converting single data points into temperature readings.
    """
    a = 0.0000132748342
    b = 0.2269469276839
    c = -241.1649627694734
    
    V = data*3.3
    #R = (36000*V)/(30-V) #will need to change for new circuit parameters
    R = (2700*V)/(3.3-V) #Based on voltage divider circuit with 2.7 k resistor
                         #in series with RTD powered on 3.3 V,
                         #reading voltage across RTD
    T = a*R**2+b*R+c
    return round(T,2)


def setup_heaters(ser, heater, level):
    """
    Sets up motor controller to control heaters. Daytime heaters will need to 
    be connected to motor one on the motor controller. Supplemental heaters will
    need to be connected to motor two.
    """
    if heater == 0: #solar simulation heaters
        sendByte = chr(0xC2)
    elif heater == 1: #nighttime heaters
        sendByte = chr(0xCA)
    ser.write(sendByte)
    ser.write(chr(level))
    
def Qi_track(filename, goal, delta, t):
    #track data from heaters
    """
    Function measures values from sensor and MCP3008 and writes temperature 
    values to a csv file at a rate of 1/w samples a second. It prints out and
    displays the values in a table in the IPython console, updating every two 
    seconds. 
    
    To display but not record data, only input a test length parameter. To record 
    data, both a test length and filename are required. New csv file is saved to
    same folder as python file RTD.py is located. 
    
    Each run through of the function adds one line to the recording file.
       
    Filename should be written as 'filename.csv'.
    """
    
    recording = False
    
    
    try: #tells you if data is recording or not
        if '.csv' in filename:
            with open(filename, 'a', newline = '') as file:
                recording = True
    except:    
        pass
        

    w = 2  #wait inbetween steps

    title = ['time(s)', 'channel']        
    header = [t, 0, 1, 2, 3, 4, 5, 6, 7]        
    
#    if t%10==0:#setting up and printing table every 10 seconds
#        if not recording:
#            print('DATA NOT RECORDING...')
#        print('|{0:^7}|{1:^7}|'.format(*title))
#        print('|{0:^7}|{1:^7}|{2:^7} |{3:^7}|{4:^7}|{5:^7}|{6:^7}|{7:^7}|{8:^7}|'
#        .format(*header))
#        print('-'*72)  
#        print("")
#        
    for a in range(t):
        temp = [0]
        temp[0] = a
	
        for i in range(0,8): #recording new values
           if i !=2 and i != 3:
          	 temp.append(rtd.get_temp(i))
	
        
        #printing out values every two seconds
        if a%2 ==0:        
                #.format(*RTD_val)) #0-1023 value from MCP3008
            print('|t={0:^5}|{1:^7}|{2:^7}|{3:^7}|{4:^7}|{5:^7}|{6:^7}|'
            .format(*temp)) #temperature calculated from raw data.
                
            print("_"*72)
            print("")     
        
        #recording values into csv if file name provided        
        if recording:
            
            with open(filename, 'a', newline = '') as file:
                dat = csv.writer(file)
                dat.writerow(temp)
            sum_temps = 0.0
            for i in range(1,7):
                    sum_temps += temp[i]
                
            average_temp = sum_temps/6.0
            print('Average temperature of the board is: ', average_temp)
            bang_bang(average_temp, goal, delta) #turns on/off heaters as necessary
            a+=w #keeps track of step
            sleep(w) #sleeps beforen next iteration    

def bang_bang(temp, goal, delta):
    """
    Runs thermal control for desired number of cycles using desired method, 
    square wave or sine wave heating. 
    """
    error = temp - goal
    if error >= 0 :
        heater.ChangeDutyCycle(0)
        #setup_heaters(ser, 0, 0) #turns off heaters, average temp too warm
        
    elif error+delta >= 0 :
        heater.ChangeDutyCycle(0)
       # setup_heaters(ser, 0, 0) #turns off heaters, average temp within margin delta
    else :
        heater.ChangeDutyCycle(100)
       # setup_heaters(ser, 0, 50) #turns on heaters, 50% duty cycle
try:
    print('trial started')
    Qi_track(filename, 35, 2, 55500)
except KeyboardInterrupt:
    print ('\n')
finally:
    GPIO.cleanup()
    print('cleanup complete have a nice day')
