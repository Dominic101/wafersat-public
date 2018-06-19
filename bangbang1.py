# -*- coding: utf-8 -*-
"""
Created on Mon Jun 18 10:38:42 2018

@author: Karolina Podsada and Dominic Maggio 

Bang-Bang Controller 
Simple thermal controller

Records the temperature of the 8 RTD sensors. Based on the average temperature of 
sensors, the algorithm turns the heaters on or off (100% duty cycle)
"""
import sh
import time
import RPi.GPIO as GPIO
import csv
from time import sleep
import subprocess
from datetime import datetime
import newRTD_V2 as rtd 
import os

#Setting up the heaters (both heaters are connected to pin 25 right now)
GPIO.setmode(GPIO.BCM) 
GPIO.setwarnings(False)
GPIO.setup(25,GPIO.OUT) #in the future, pin24 will also be an out for the heaters
heater = GPIO.PWM(25,20) #frequency is 20 (?)
heater.start(0) #starts the heaters, duty cycle 0
current_DC = 0.0
previous_error = 0


# filename formatting
time = datetime.now()
time = time.strftime("%Y-%m-%d %H:%M")
filename = 'pi_stats' + time + '.csv'
filename = filename.replace(' ', '')
print('Filename for generated file will be:')
print(filename)


    
def Qi_track(filename, goal, delta, t, control_alg):
    """
    Function measures values from sensor and MCP3008 and writes temperature 
    values to a csv file at a rate of 1/w samples a second. It prints out and
    displays the values in a table in the IPython console, updating every four 
    seconds. 
    
    To display but not record data, only input a test length parameter. To record 
    data, both a test length and filename are required. New csv file is saved to
    same folder as python file RTD.py is located. 
    
    Each run through of the function adds one line to the recording file.
       
    Filename should be written as 'filename.csv'.
    Calls the controller (Bang-Bang)
    
    filename: name of the file to write to (automatically generated at the top)
    goal: the goal temperature
    delta: the margin
    t: how many seconds to run this for
    """
    
    recording = False
    
    
    try: #tells you if data is recording or not
        if '.csv' in filename:
            with open(filename, 'a', newline = '') as file:
                recording = True
    except:    
        pass
        
    w = 2  #wait inbetween steps
    
    for a in range(t):
        temp = [0]
        temp[0] = a
	
        for i in range(0,8): #recording new temperature values of the RTDs 
           if i !=2 and i != 3:
          	 temp.append(rtd.get_temp(i))
	
        cpu_temp = str(float(sh.cat('/sys/class/thermal/thermal_zone0/temp')) / 1000)
        temp.append(cpu_temp)
        temp.append(current_DC)
        
        
        #printing out values every two seconds
        if a%4 == 0:        
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
                
            #Calculating the average temperature of the board
            sum_temps = 0.0
            for i in range(1,7):
                    sum_temps += temp[i]    
            average_temp = sum_temps/6.0
            
            
            print('Average temperature of the board is: ', average_temp)
            current_DC=control_alg(average_temp, goal, delta) #turns on/off heaters as necessary
            print('Current duty cycle is set to: ', current_DC)
            
            
            a+=w #keeps track of step
            sleep(w) #sleeps beforen next iteration    

def bang_bang(temp, goal, delta):
    """
    Bang-Bang Controller: Turns heaters off or on 100% duty cycle if the average temp of the board is below the margin.
    
    temp: current temp (average temp of the RTDs)
    goal: the goal temp
    delta: margin
    
    """
    error = temp - goal
    if error >= 0 :
        heater.ChangeDutyCycle(0)
        current_DC = 0.0
        #setup_heaters(ser, 0, 0) #turns off heaters, average temp too warm
        
    elif error+delta >= 0 :
        heater.ChangeDutyCycle(0)
        current_DC = 0.0
       # setup_heaters(ser, 0, 0) #turns off heaters, average temp within margin delta
    else :
        heater.ChangeDutyCycle(100)
        current_DC = 100
       # setup_heaters(ser, 0, 50) #turns on heaters, 100% duty cycle
        return current_DC

def bang_bang2(temp,goal,delta):
    '''
    brings temp up to desired value and then turns heaters on when it falls below a delta range
    '''
    error=temp-goal
    if error>=0:
        heater.ChangeDutyCycle(0)
        current_DC = 0.0
    elif goal-temp>=delta:
        heater.ChangeDutyCycle(100)  
        current_DC = 100
    return current_DC

def PD(temp, goal, delta):
    
    if goal-temp <= 0:
        heater.ChangeDutyCycle(0.0)
        return 0.0
    else: 
        #tuning coefficients
        proportional_gain_value = 0.82
        derivative_gain_value = 0.40
    
        error = goal-temp
        derivative_error = error - previous_error
        
        global previous_error = error
        
        D = derivative_gain_value*derivative_error
        P = proportional_gain_value*error
        PD_output = P + D
        
        if PD_output < 0 :
            PD_output = 0.0
        if PD_output > 100:
            PD_output = 100
        heater.ChangeDutyCycle(PD_output)
        return PD_output
    
    
    
try:
    print('trial started')
    Qi_track(filename, 35, 2, 55500, bang_bang2)
except KeyboardInterrupt:
    print ('\n')
finally:
    GPIO.cleanup()
    print('cleanup complete have a nice day')
 