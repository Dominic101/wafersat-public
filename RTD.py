# -*- coding: utf-8 -*-
"""
Author: Raul Largaespada
For WaferSat UROP Summer 2017

Python file for using Raspberry Pi as a data acquisition unit for measuring 
temperatures using RTDs. 

Note that this file assumes that all 8 channels are functioning and connected 
to RTDs. If not all channels are being used, specify how many channels are 
being used in the plot_temp function and make sure your circuit is only using 
the first i channels on the MCP3008, where i is the number of desired channels.
"""

from gpiozero import LED, MCP3008
from time import sleep
import csv
import pylab as plt

led = LED(15)

RTD0 = MCP3008(channel=0, clock_pin=18, mosi_pin=24, miso_pin=23, select_pin=25)
RTD1 = MCP3008(channel=1, clock_pin=18, mosi_pin=24, miso_pin=23, select_pin=25)
RTD2 = MCP3008(channel=2, clock_pin=18, mosi_pin=24, miso_pin=23, select_pin=25)
RTD3 = MCP3008(channel=3, clock_pin=18, mosi_pin=24, miso_pin=23, select_pin=25)
RTD4 = MCP3008(channel=4, clock_pin=18, mosi_pin=24, miso_pin=23, select_pin=25)
RTD5 = MCP3008(channel=5, clock_pin=18, mosi_pin=24, miso_pin=23, select_pin=25)
RTD6 = MCP3008(channel=6, clock_pin=18, mosi_pin=24, miso_pin=23, select_pin=25)
RTD7 = MCP3008(channel=7, clock_pin=18, mosi_pin=24, miso_pin=23, select_pin=25)

RTD_list = [RTD0, RTD1, RTD2, RTD3, RTD4, RTD5, RTD6, RTD7]
#RTD_val = [RTD0.value, RTD1.value, RTD2.value, RTD3.value, RTD4.value, 
#           RTD5.value, RTD6.value, RTD7.value]

led.on()
sleep(2)
led.off()
del led

def comm_test(): #simple communication test that flashes LED.
    led = LED(15)    
    led.blink(.5,.5)
    sleep(3)
    led.off()
    del led
    return

def TFD(data): 
    """
    For converting single data points into temperature readings.
    """
    a = 0.0000132748342
    b = 0.2269469276839
    c = -241.1649627694734
    
    V = data*3.3
    R = (36000*V)/(30-V) #will need to change for new circuit parameters
    T = a*R**2+b*R+c
    return round(T,2)

def collect_data(test_length,filename=None):
    """
    Function measures values from sensor and MCP3008 and writes temperature 
    values to a csv file at a rate of 1/w samples a second. It prints out and
    displays the values in a table in the IPython console, updating every two 
    seconds. 
    
    To display but not record data, only input a test length parameter. To record 
    data, both a test length and filename are required. New csv file is saved to
    same folder as python file RTD.py is located. 
       
    Filename should be written as 'filename.csv'.
    """
    
    recording = False
    
    
    try: #tells you if data is recording or not
        if '.csv' in filename:
            with open(filename, 'a', newline = '') as file:
                recording = True
    except:    
        pass
        
    t = 0
    w = 1

    while t<= test_length:
        title = ['time(s)', 'channel']        
        header = [t, 0, 1, 2, 3, 4, 5, 6, 7]        
        
        if t%10==0:#setting up and printing table
            if not recording:
                print('DATA NOT RECORDING...')
            print('|{0:^7}|{1:^7}|'.format(*title))
            print('|{0:^7}|{1:^7}|{2:^7} |{3:^7}|{4:^7}|{5:^7}|{6:^7}|{7:^7}|{8:^7}|'
            .format(*header))
            print('-'*72)  
            print("")
            
        RTD_val = [t]
        temp = ["deg C."]
        for i in RTD_list: #recording new values
            RTD_val.append(int((i.value)*1024))
            temp.append(TFD(i.value))
        
        
        #printing out values every two seconds
        if t%2 ==0:        
            print('|t={0:^5}|C0:{1:^4}|C1:{2:^4}|C2:{3:^4}|C3:{4:^4}|C4:{5:^4}|\
C5:{6:^4}|C6:{7:^4}|C7:{8:^4}'
                .format(*RTD_val)) #0-1023 value from MCP3008
            print('|{0:^7}|{1:^7}|{2:^7}|{3:^7}|{4:^7}|{5:^7}|{6:^7}|{7:^7}|{8:^7}'
            .format(*temp)) #temperature calculated from raw data.
                
            print("_"*72)
            print("")     
        
        #recording values into csv if file name provided        
        if recording:
            temp[0]=t
            with open(filename, 'a', newline = '') as file:
                dat = csv.writer(file)
                dat.writerow(temp)
        
        
        
        t+=w
        sleep(w)
        
def calibration_test(test_length,filename=None):
    """
    For calibrating Pi MCP3008 with signal generator. Modification of collect_data.
    Records and prints voltages instead of temperatures. 
    """
    
    recording = False
    
    
    
    try: #tells you if data is recording or not
        if '.csv' in filename:
            with open(filename, 'a', newline = '') as file:    
                recording = True
    except:    
        pass
        
    t = 0
    w = .5

    while t<= test_length:
        title = ['time(s)', 'channel']        
        header = [t, 0, 1, 2, 3, 4, 5, 6, 7]        
        
        if t%10==0:#setting up and printing table
            if not recording:
                print('DATA NOT RECORDING...')
            print('|{0:^7}|{1:^7}|'.format(*title))
            print('|{0:^7}|{1:^7}|{2:^7} |{3:^7}|{4:^7}|{5:^7}|{6:^7}|{7:^7}|{8:^7}|'
            .format(*header))
            print('-'*72)  
            print("")
            
        RTD_val = [int(t)]
        vol = ["Voltage"]
        for i in RTD_list: #recording new values
            RTD_val.append(int((i.value)*1024))
            vol.append(round((i.value)*3.3,3))
        
        
        #printing out values every two seconds
        if t%2 ==0:        
            print('|t={0:^5}|C0:{1:^4}|C1:{2:^4}|C2:{3:^4}|C3:{4:^4}|C4:{5:^4}|\
C5:{6:^4}|C6:{7:^4}|C7:{8:^4}'
                .format(*RTD_val)) #0-1023 value from MCP3008
            print('|{0:^7}|{1:^7}|{2:^7}|{3:^7}|{4:^7}|{5:^7}|{6:^7}|{7:^7}|{8:^7}'
            .format(*vol)) #temperature calculated from raw data.
                
            print("_"*72)
            print("")     
        
        #recording values into csv if file name provided        
        if recording:
            vol[0]=t
            with open(filename, 'a', newline = '') as file:
                dat = csv.writer(file)
                dat.writerow(vol)
        
        
        
        t+=w
        sleep(w)

def plot_temp(filename, chn=8):
#    x = [] 
#    y = []
#    with open(filename, 'r', newline = '') as file:
#        reader = csv.reader(file)
#               
#        for row in reader:
#            x.append(int(row[0]))
#        for i in range(1,chn+1):
#            # y = []
#            print(i)
#            for row in reader:
#                y.append(row[i])
#                #print(y)
#            print(y)
#            #plt.plot(x,y)
#            
    pass
            


"""To do:create a better generator function, output to csv format, connect 
remotely pi and get data from there, plot from here, return the maximum and 
minimum values from each channel.  
"""
