#-*- coding: utf-8 -*-
"""
Created on Mon Jun 18 10:38:42 2018

@author: Karolina Podsada and Dominic Maggio and George Chen 

Bang-Bang Controller 
Simple thermal controller
PID Controller

Records the temperature of the 8 RTD sensors. Based on the average temperature of 
sensors, the algorithm turns the heaters on or off (100% duty cycle)
"""

from time import sleep
import time as timey
import RPi.GPIO as GPIO
import csv
import math
from datetime import datetime
import current_RTD_quar as rtd

x_old=None # for laplace filter 
integral=0 # I term for PID controller
current_DC = 0.0 # duty cycle
previous_error = 0 # for taking the derivative error
derivative_errors = [0,0,0,0] # keeps track of last four derivative errors
fixit = 0.0 # "fixit" term for PD controller
numloops = 0 # iterator
delta_t = 0.5 # Time between sampling. SET THIS HERE

#Setting up the heaters (both heaters are connected to pin 25 right now)
GPIO.setmode(GPIO.BCM) 
GPIO.setwarnings(False)
GPIO.setup(25,GPIO.OUT) #Use pin25 for both heaters (pin24 is broken in rev1.2)
heater = GPIO.PWM(25,20) #frequency is 20Hz, pin is 25
heater.start(0) #starts the heaters, duty cycle 0


    
def run(goal, time, control_alg, cool_down):
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
    global x_old
    global current_DC
    global numloops
    global delta_t
    start_time = timey.time()
    timeforfile = datetime.now()
    timeforfile = timeforfile.strftime("%Y-%m-%d - %H:%M") 
    filename = timeforfile + str(control_alg.__name__) + '.csv'
    filename = filename.replace(' ', '')
    
    with open(filename, 'a', newline = '') as file:
        pass

    current_DC = 0

    while(True):
        loop_start = timey.time()
        datarow = [0]  
        current_time=abs(start_time-timey.time())
        datarow[0] = round(current_time,2)
	
        for i in range(len(rtd.RTD_list)): #recording new temperature values of the RTDs 
            if i!=2 and i!=4: #channel 2 on Rev1.2 is not functional. Channel 4 is being used for the cold plate. Ignoring both        
                datarow.append(rtd.get_temp(i))
	
       
        #printing out values every two seconds
        if numloops%4 == 0:  
            print('t = ',datarow[0])
            ##  #.format(*RTD_val)) #0-1023 value from MCP3008
            print('|t={0:^5}|{1:^7}|{2:^7}|{3:^7}|{4:^7}|{5:^7}|{6:^7}'.format(*datarow)) #temperature calculated from raw data.


                
        #Calculating the average temperature of the board
        sum_temps = 0.0
        for i in range(1,len(datarow)):
                sum_temps += datarow[i]    
        average_temp = sum_temps/6.0 #divide by number of RTDs on board
        datarow.append(average_temp)
        if numloops==0:
            x_old=average_temp #to make the filter work
        filtered_temp = laplacefilter(average_temp)
        datarow.append(filtered_temp)
        print('Filtered temp: ', filtered_temp)
        print('Average temp: ', average_temp)
        plate_temp = rtd.get_temp(4) #currently the cold plate RTD is connected to channel 4
        print('Cold plate temp: ', plate_temp)
        datarow.append(plate_temp)
        stop_heat=False
        if current_time>=time:
            stop_heat=True
        if not stop_heat:
            if abs(filtered_temp-average_temp)<.5: #checking if filter has caught up to actual data
                current_DC=control_alg(filtered_temp, goal)
            else: 
                current_DC=control_alg(average_temp, goal)

        elif cool_down and stop_heat:
            heater.ChangeDutyCycle(0)
            current_DC=0 
            if current_time>=time+30: #cooldown is set to 30 seconds
                raise KeyboardInterrupt('cool_down complete')
        else:
            raise KeyboardInterrupt('heat test done')
        print('Current DC:', current_DC)
        
        datarow.append(current_DC)

        with open(filename, 'a', newline = '') as file:
            dat = csv.writer(file)
            dat.writerow(datarow)
        
        print("_"*72)
        print("")              
        
        loop_end = timey.time()
        sleep(delta_t - (loop_end - loop_start)) # Sleeps for exactly delta_t minus code runtime
        numloops += 1
        

def bang_bang(temp, goal):
    """
    Bang-Bang Controller: Turns heaters off or on 100% duty cycle if the average temp of the board is below the margin.
    
    temp: current temp (average temp of the RTDs)
    goal: the goal temp
    delta: margin
    
    """
    error = temp - goal
    delta = 2
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

def bang_bang2(temp,goal):
    '''
    Brings temp up to desired value and then turns heaters on when it falls below a delta range. 
    Cretes a sawtooth average temperature.
    '''
    global current_DC
    error=temp-goal
    delta = 2
    if error>=0:
        heater.ChangeDutyCycle(0)
        current_DC = 0.0
    elif goal-temp>=delta:
        heater.ChangeDutyCycle(100)  
        current_DC = 100
    return current_DC

def P(temp, goal):
    '''
    Simple P controller. Does not use the sigmoid function to map the duty cycle between 0-100. 
    '''
    global previous_error
    if goal-temp <= 0:
        heater.ChangeDutyCycle(0.0)
        return 0.0
    else: 
        #tuning coefficients
        proportional_gain_value = 5
    
        error = goal-temp
        P = proportional_gain_value*error
        
        if P < 0 :
            P = 0.0
        if P > 100:
            P = 100
        heater.ChangeDutyCycle(P)
        return P

    
def P2(temp,want, Kp=3.3):
    '''
    Second P controller. This version uses the sigmoid function to map the duty cycle between 0-100. 
    This is the current version of the P controller (use this one)
    '''
    global current_DC 

    if want-temp<0:
        error = 0 #turns off controller if average temperature is above the target temperature
    else:
        error = want-temp #check if it should be the other way around
    PID_sum = error * Kp       

    if PID_sum!=0:
        current_DC = sigmoid(PID_sum)
        heater.ChangeDutyCycle(current_DC)
    else:
        current_DC = 0
        heater.ChangeDutyCycle(0)
    return current_DC


def update_derivatives(new_der):
    '''
    This method updates the derivative list with a new derivative error. 
    '''
    global derivative_errors
    derivative_errors.pop(0)
    derivative_errors.append(-new_der) 
    #print(derivative_errors,'der errors')


def get_average_der():
    '''
    Returns the average derivative error from the list of four
    '''
    global derivative_errors
    return sum(derivative_errors) / 4.0


def PD(temp, want, Kp=2):
    '''
    Not actually a PD controller. This is a controller we made up involving a "fixit" term.
    '''
    global current_DC
    global fixit
    global numloops
    global previous_error
    global delta_t
    error = want - temp 

    if error < 0:
        d_error = (error - previous_error) / delta_t
        previous_error = error
#        update_derivatives(d_error)
        current_DC = 0.0
        heater.ChangeDutyCycle(current_DC)
        return current_DC
    else:
        Pt = error * Kp

        d_error = (error - previous_error) / delta_t
        previous_error = error
        update_derivatives(d_error)
        if numloops % 2 == 0 and numloops!=0:
            if get_average_der() < 0.06 and error >.2:
                fixit += 0.1
            
            elif get_average_der() > 0.2 and numloops>60:
                if fixit>=0:
                    fixit -= 0.5
            elif get_average_der() > .1:
                if fixit>=0:
                    fixit -= 0.1
        if error>.2:
            PID_sum = Pt + fixit*error
        else:
            PID_sum=Pt+fixit*error
        current_DC = sigmoid(PID_sum)
        heater.ChangeDutyCycle(current_DC)
        return current_DC


def PID(temp,want, Ki=.1, Kp=2, Kd=1):
    '''
    PID controller. Set proportionality constants manually. 
    '''
    global current_DC 
    global integral
    global numloops
    global previous_error
    global delta_t
    error = want - temp
    if numloops >= 50: #prevent integral windup in the beginning
        integral += error*delta_t
    print('integral =', integral)
    d_error = (error - previous_error) / delta_t
    previous_error = error
    update_derivatives(d_error)
    if numloops>=2: #make sure there are four derivatives in the derivative list before using
        av_dev=get_average_der()
    else:
        av_dev=0
    PID_sum=(error*Kp)+(integral*Ki)+av_dev*Kd
    print('PIDSum = ',PID_sum)
    current_DC = sigmoid(PID_sum)
    heater.ChangeDutyCycle(current_DC)
    return current_DC
    
def sigmoid(PID_sum):
    '''
    Take in the value from the PID function and puts it into the sigmoid
    function which is shifted four units to the right. The result is then scaled 
    by 100.
    '''
    funct_shift=PID_sum-4
    sigmoid=math.e**funct_shift/(math.e**funct_shift+1)
    return sigmoid*100

def cooling(temp, goal) :
    '''
    Heaters off
    '''
    current_DC =0.0
    heater.ChangeDutyCycle(current_DC)
    return current_DC
    
    
def laplacefilter(avg_temp, alpha = .3):
    '''
    See google drive for Miller's derivation.
    '''
    global x_old
    global delta_t
    x_new=(1-alpha*delta_t)*x_old+delta_t*alpha**.5*avg_temp
    o=alpha**.5*x_new
    x_old=x_new
    return o
    
try:
    print('test started')
    '''
    Arguments for run() : goal temp, seconds to run with heat, control_alg, cool_down boolean (set to 30 seconds, change in run())
    '''
    run(35, 120, bang_bang2, cool_down=True)
except KeyboardInterrupt:
    print ('\n')
finally:
    print('cleanup complete have a nice day')
 
