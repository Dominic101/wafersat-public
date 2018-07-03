#-*- coding: utf-8 -*-
"""
Created on Mon Jun 18 10:38:42 2018

@author: Karolina Podsada and Dominic Maggio and George Chen 

Bang-Bang Controller 
Simple thermal controller

Records the temperature of the 8 RTD sensors. Based on the average temperature of 
sensors, the algorithm turns the heaters on or off (100% duty cycle)
"""
import sh
from time import sleep
import time as timey
import RPi.GPIO as GPIO
import csv
import subprocess
from datetime import datetime
import current_RTD as rtd 
import os
from numpy import mean,math  
x_old=None
integral=0
#Setting up the heaters (both heaters are connected to pin 25 right now)
GPIO.setmode(GPIO.BCM) 
GPIO.setwarnings(False)
GPIO.setup(25,GPIO.OUT) #in the future, pin24 will also be an out for the heaters
heater = GPIO.PWM(25,20) #frequency is 20 (?)
heater.start(0) #starts the heaters, duty cycle 0
current_DC = 0.0
previous_error = 0
derivative_errors = [0,0,0,0]
fixit = 0.0
a = 0

    
def Qi_track(goal, delta, t, control_alg, cool_down):
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
    recording = False
    start_time = timey.time()
    timeforfile = datetime.now()
    timeforfile = timeforfile.strftime("%Y-%m-%d - %H:%M") 
    filename =  str(control_alg.__name__) + timeforfile + '.csv'
    filename = filename.replace(' ', '')
    
    try: #tells you if data is recording or not
        if '.csv' in filename:
            with open(filename, 'a', newline = '') as file:
                recording = True
    except:    
        pass
        
    w = .5  #wait inbetween steps
    current_DC = 0
    global a
    for a in range(t*100000):#t*100000 insures that input time dictates time of test
        temp = [0]  
        current_time=abs(start_time-timey.time())
        temp[0] = round(current_time,2)
	
        for i in range(0,8): #recording new temperature values of the RTDs 
           if i !=4 and i!=6:
           	temp.append(rtd.get_temp(i))
	
#        cpu_temp = str(float(sh.cat('/sys/class/thermal/thermal_zone0/temp')) / 1000)
#        temp.append(cpu_temp)
        #temp.append(current_DC)
        
        
        #printing out values every two seconds
        if a%4 == 0:  
            print('t= ',temp[0])
              ##  #.format(*RTD_val)) #0-1023 value from MCP3008
            print('|t={0:^5}|{1:^7}|{2:^7}|{3:^7}|{4:^7}|{5:^7}|{6:^7}'
           .format(*temp)) #temperature calculated from raw data.

        #recording values into csv if file name provided        
        if recording:
                
            #Calculating the average temperature of the board
            sum_temps = 0.0
            for i in range(1,7):
                    sum_temps += temp[i]    
            average_temp = sum_temps/6.0
            temp.append(average_temp)
            if a==0:
                x_old=average_temp
            filtered_temp=davefilter(average_temp)
            temp.append(filtered_temp)
            print('Filtered temp: ', filtered_temp)
           # global current_DC
            print('Average temp: ', average_temp)
            stop_heat=False
            if current_time>=t:
                stop_heat=True
            if not stop_heat:
                if a<25:
                    if abs(filtered_temp-average_temp)<.5:
                        current_DC=control_alg(filtered_temp, goal)
                    else:
                        current_DC=control_alg(average_temp, goal)
                else:
                    if abs(filtered_temp-average_temp)<1.5:
                        current_DC=control_alg(filtered_temp, goal) #turns on/off heaters as necessary
                    else:
                        current_DC=control_alg(average_temp, goal)
            elif cool_down and stop_heat:
                heater.ChangeDutyCycle(0)
                current_DC=0 
                if current_time>=t+30:
                    raise KeyboardInterrupt('cool_down complete')
            else:
                raise KeyboardInterrupt('heat test done')
            print('Current DC:', current_DC)
            
            temp.append(current_DC)

            with open(filename, 'a', newline = '') as file:
                dat = csv.writer(file)
                dat.writerow(temp)
            
            print("_"*72)
            print("")              
            a+=w #keeps track of step
            sleep(w) #sleeps before next iteration    
        

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
    brings temp up to desired value and then turns heaters on when it falls below a delta range
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
    global previous_error
    if goal-temp <= 0:
        heater.ChangeDutyCycle(0.0)
        return 0.0
    else: 
        #tuning coefficients
        proportional_gain_value = 5
    
        error = goal-temp
        P = proportional_gain_value*error
        PD_output = P
        
        if PD_output < 0 :
            PD_output = 0.0
        if PD_output > 100:
            PD_output = 100
        heater.ChangeDutyCycle(PD_output)
        return PD_output

    
def P2(temp,want, Kp=3.3):
    global current_DC 
    def get_error(temp,want,delta):
       # if abs(want-temp)<delta or temp-want>0:
       if want-temp<0:
            return 0
       else:
            return want-temp#check if it should be the other way around

    def P_control(Kp):
        Pt = get_error(temp,want,delta) * Kp
        return Pt
    PID_sum=P_control(Kp)
    if PID_sum!=0:
        current_DC = sigmoid(PID_sum)
        heater.ChangeDutyCycle(current_DC)
    else:
        current_DC = 0
        heater.ChangeDutyCycle(0)
    return current_DC


def update_derivatives(new_der):
    global derivative_errors
    derivative_errors.pop(0)
    derivative_errors.append(-new_der)
    print(derivative_errors,'der errors')


def get_average_der():
    global derivative_errors
    return sum(derivative_errors) / 4.0


def PD(temp, want, Kp=2):
    '''
    This is not really what we want but close. D should only execute every 2 seconds not every time PD is called.
    '''
    global current_DC
    global fixit
    global a
    global previous_error
    error = want - temp 

    if error < 0:
        d_error = (error - previous_error) / 0.5
        previous_error = error
#        update_derivatives(d_error)
        current_DC = 0.0
        heater.ChangeDutyCycle(current_DC)
        return current_DC
    else:
        Pt = error * Kp

        d_error = (error - previous_error) / 0.5
        previous_error = error
        update_derivatives(d_error)
        if a % 2 == 0 and a!=0:
            if get_average_der() < 0.06 and error >.2:
                fixit += 0.1
            
            elif get_average_der() > 0.2 and a>60:
                if fixit>=0:
                    fixit -= 0.5
            elif get_average_der() > .1:
                if fixit>=0:
                    fixit -= 0.1
        print('fixit: ', fixit)
        if error>.2:
            PID_sum = Pt + fixit*error
        else:
            PID_sum=Pt+fixit*error
        current_DC = sigmoid(PID_sum)
        heater.ChangeDutyCycle(current_DC)
        return current_DC


def PI(temp,want, Ki=.1, Kp=2):
    global current_DC 
    global integral
    global a
    error = want - temp
    if a >= 50:
        integral += error*0.5
    print('integral =', integral)
    if 3 < 0:
        current_DC = 0.0
        heater.ChangeDutyCycle(current_DC)
        return current_DC
    else : 
#        PID_sum=error*Kp+integral*Ki
        if integral>0:
            PID_sum=(error*Kp)+(integral*Ki)
        else:
             PID_sum=error*Kp
        print('PIDSum = ',PID_sum)
        current_DC = sigmoid(PID_sum)
        heater.ChangeDutyCycle(current_DC)
        return current_DC


def PID(temp,want, Ki=.1, Kp=2, Kd=1):
    global current_DC 
    global integral
    global a
    global previous_error
    error = want - temp
    if a >= 50:
        integral += error*0.5
    print('integral =', integral)
    d_error = (error - previous_error) / 0.5
    previous_error = error
    update_derivatives(d_error)
    if a>=2:
        av_dev=get_average_der()
    else:
        av_dev=0
    PID_sum=(error*Kp)+(integral*Ki)+av_dev*Kd
    print('PIDSum = ',PID_sum)
    current_DC = sigmoid(PID_sum)
    heater.ChangeDutyCycle(current_DC)
    return current_DC


def fail_safe():
    ''' implement if needed
    '''
    heater.ChangeDutyCycle(0)
    
def sigmoid(PID_sum):
    '''
    take in the value from the PID function and puts it into the sigmoid
    function which is shifted four units to the right. The result is then scaled 
    by 100.
    '''
    funct_shift=PID_sum-4
    sigmoid=math.e**funct_shift/(math.e**funct_shift+1)
    return sigmoid*100

   
    

#def PID(temp,want, delta, Kp=2,Ki=.2,Kd=1.3,It=0,previous_error=0):
#    def get_error(temp,want,delta):
#        temp_avg = mean(temp[1:])
#        if abs(want-temp_avg)<delta:
#            return 0
#        else:
#            return want-temp_avg#check if it should be the other way around
#    error=get_error(temp,want,delta)
#    def P_control(Kp):
#        Pt = error * Kp
#        return Pt
#    
#    
##    def I_control(It, Ki):
##        It = It + error
##        It = It * Ki
##        return It
#    
#    
#    def D_control(previous_error, Kd):
#        Dt = (error-previous_error) * Kd
#        # print(current_control_output_value, previous_control_output_value, Dt)
#        return Dt
#    previous_error=error
#    PID_sum=P_control(Kp)+D_control(previous_error, Kd)
#    heater.ChangeDutyCycle(sigmoid(PID_sum))
#def fail_safe():
#    ''' implement if needed
#    '''

#    heater.ChangeDutyCycle(0)
#    
#def sigmoid(PID_sum):
#    '''
#    take in the value from the PID function and puts it into the sigmoid
#    function which is shifted four units to the right. The result is then scaled 
#    by 100.
#    '''
#    funct_shift=PID_sum-4
#    sigmoid=math.e**funct_shift/(math.e**funct_shift+1)
#    return sigmoid*100


#def PD(temp, goal, delta):
#    global previous_error
#    if goal-temp <= 0:
#        heater.ChangeDutyCycle(0.0)
#        return 0.0
#    else: 
#        #tuning coefficients
#        proportional_gain_value = 0.82
#        derivative_gain_value = 0.40
#    
#        error = goal-temp
#        derivative_error = error - previous_error
#        
#        previous_error = error
#        
#        D = derivative_gain_value*derivative_error
#        P = proportional_gain_value*error
#        PD_output = P + D
#        
#        if PD_output < 0 :
#            PD_output = 0.0
#        if PD_output > 100:
#            PD_output = 100
#        heater.ChangeDutyCycle(PD_output)
#        return PD_output
    
def davefilter(avg_temp, a=.3, delta_t=.5):
    global x_old
    x_new=(1-a*delta_t)*x_old+delta_t*a**.5*avg_temp
    o=a**.5*x_new
    x_old=x_new
    return o
    
try:
    print('trial started')
    #filename, goal temp, delta, seconds to run with heat, control_alg, 
    Qi_track(34, 2, 500, PI, cool_down=True)
except KeyboardInterrupt:
    print ('\n')
finally:
    print('cleanup complete have a nice day')
 
