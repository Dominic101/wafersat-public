"""
Author: Raul Largaespada, modified by Charles Lindsy on 2.01.18 for PCBSat Rev1
modified by Dominic Maggio and Karolina Podsada

For WaferSat UROP Thermal Test Circuit with PCBSat Rev1

Python file for using Raspberry Pi as a data acquisition unit for measuring 
temperatures using RTDs. 
"""

from gpiozero import MCP3208

adc = MCP3208()


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
        #R=(3600*V)/(30-V) this is the old circuit parameters

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
    
    #reading = RTD_list[RTD].value #gets ADC reading from selected RTD, a float range 0-1
    reading = float(adc.read(RTD)/4096)
    temp = TFD(reading)
    
    return temp

