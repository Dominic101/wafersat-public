""" 
Author: Raul Largaespada, modified by Charles Lindsy on 2.01.18 for PCBSat Rev1
For WaferSat UROP Thermal Test Circuit with PCBSat Rev1
Python file for using Raspberry Pi as a data acquisition unit for measuring 
temperatures using RTDs. 
Note that this file assumes that all 8 channels are functioning and connected 
to RTDs. If not all channels are being used, specify how many channels are 
being used in the plot_temp function and make sure your circuit is only using 
the first i channels on the MCP3208, where i is the number of desired channels.
"""

from gpiozero import MCP3208
from time import sleep
import time
import csv
import RPi.GPIO as GPIO
#led = LED(21) #initializes status LED connected to GPIO21
adc = MCP3208()

RTD0 = MCP3208(channel=0, port=0, device=0) #ADC input 0 settings using MCP3208 hardware interface (MOSI: GPIO10, MISO: GPIO9, CLK: GPIO11, CS0: GPIO8)
RTD1 = MCP3208(channel=1, port=0, device=0) 
RTD2 = MCP3208(channel=2, port=0, device=0)
RTD3 = MCP3208(channel=3, port=0, device=0)
RTD4 = MCP3208(channel=4, port=0, device=0)
RTD5 = MCP3208(channel=5, port=0, device=0)
RTD6 = MCP3208(channel=6, port=0, device=0)
RTD7 = MCP3208(channel=7, port=0, device=0)
#
RTD_list = [RTD0, RTD1, RTD2, RTD3, RTD4, RTD5, RTD6, RTD7]

#led.on()
#sleep(2)
#led.off()

#del led

def comm_test(): #simple communication test that flashes LED.
    led = LED(21)    
    led.blink(.5,.5)
    sleep(3)
    led.off()
    del led
    return

def TFD(data): 
    """
    For converting single data points into temperature readings.
    Data should be a float value from 0 to 1, as read by ADC with Vref at 3.3 V
    """
    a = 0.0000132748342
    b = 0.2269469276839
    c = -241.1649627694734

    if data != 1:
        V = data*3.3
        R = (2700*V)/(3.3-V) #Based on voltage divider circuit with 2.7 k resistor
                         #in series with RTD powered on 3.3 V,

    else: #in case of dividing by 0
        R = 100000 #arbitrary value for extremely high R (most likely
        #due to error)

        
    T = a*R**2+b*R+c
    return round(T,2)

def get_temp(RTD):
    '''
    Returns current temperature of an RTD
    RTD should be int range 0 to 7 corresponding to RTD channels
    '''
    
    reading = RTD_list[RTD].value #gets ADC reading from selected RTD, a float range 0-1 reading = float(adc.read(RTD)/4096)
    temp = TFD(reading)
    
    return temp

def get_voltage(RTD):
    '''
    Returns current voltage drop over an RTD, assuming Vref connected to 3.3 V
    RTD should be int range 0 to 7 corresponding to RTD channels
    '''

    reading = RTD_list[RTD].value
    voltage = reading*3.3

    return voltage
             
def collect_data(test_length,sample_rate,filename=None):
    """
    Function measures values from sensor and MCP3208 and writes temperature 
    values to a csv file at a rate of sample_rate samples per second. It prints out and
    displays the values in a table in the IPython console, updating every two 
    seconds. 
    
    To display but not record data, only input a test length parameter. To record 
    data, both a test length and filename are required. New csv file is saved to
    same folder as python file RTD.py is located. 
       
    sample_rate should be number of samples/sec
    
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
    w = 1/sample_rate #approximate delay time between samples, in seconds

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
            RTD_val.append(int((i.value)*1024)) #adds raw ADC value in range 0 to 1024
            temp.append(TFD(i.value))   #adds temperature value
        
        
        #printing out values every two seconds
        if t%2 ==0:        
            print('|t={0:^5}|C0:{1:^4}|C1:{2:^4}|C2:{3:^4}|C3:{4:^4}|C4:{5:^4}|\
C5:{6:^4}|C6:{7:^4}|C7:{8:^4}'
                .format(*RTD_val)) #0-1023 value from MCP3008
            print('|{0:^7}|{1:^7}|{2:^7}|{3:^7}|{4:^7}|{5:^7}|{6:^7}|{7:^7}|{8:^7}'
            .format(*temp)) #temperature calculated from raw data.
                
            print("_"*72)
            print("")     
        
        #recording time and temperature values into csv if file name provided        
        if recording:
            temp[0]=t
            with open(filename, 'a', newline = '') as file:
                dat = csv.writer(file)
                dat.writerow(temp)
        
        
        
        t+=w
        sleep(w)
        
def calibration_test(test_length,sample_rate,filename=None):
    """
    For calibrating Pi MCP3208 with signal generator. Modification of collect_data.
    Records and prints voltages instead of temperatures.
    To display but not record data, only input a test length parameter. To record 
    data, both a test length and filename are required. New csv file is saved to
    same folder as python file RTD.py is located. 
       
    sample_rate should be number of samples/sec
    
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
    w = 1/sample_rate

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

"""l  = []
x = 0
l2 = []
t1 = 0
t2 = 0
tvec = []
tempvec = []
#print("start collect")
#for c in range(0,1000):
rtd  = 5
for i in range(0,5*60*2): #for 1 minute
  
  #t1 = time.time()
  
  l = []
  for i in range(0,15):
     x = get_temp(rtd)
     l.append(x)	
  avg = 0
  for e in l:
      avg+=e
  avg = avg/len(l)
  
  t2 = time.time()
  tvec.append(t2)
  tempvec.append(avg)
  print(avg)
  #l2.append(avg)
  sleep(0.5)
  #print(avg)
  #sleep(0.05)
print(tvec)
print(tempvec)
#print("end collect")
#l = []
#l = l2
#min  = l[1]
#max = l[1]
"""
"""for i in range(1,len(l)):
	if l[i] < l[i-1]:
		min = l[i]
for i in range(1,len(l)):
        if l[i] > l[i-1]:
                max = l[i]
print(len(l))
print("min = ", min)
print("max = ", max)
print("difference = ", max-min)
print("avg = ", avg)
print("t1 = ", t1)
print("time to process = ", t2-t1)
"""
"""To do:create a better generator function, output to csv format, connect 
remotely pi and get data from there, plot from here, return the maximum and 
minimum values from each channel.  
"""

