# -*- coding: utf-8 -*-
"""
Created on Thu Jun 21 10:37:19 2018
 
Author: Raul Largaespada, modified by Charles Lindsy on 2.01.18 for PCBSat Rev1
For WaferSat UROP Thermal Test Circuit with PCBSat Rev1
Modified by Karolina Podsada and Dominic Maggio and George Chen on 08.07.2018


Python file for using Raspberry Pi as a data acquisition unit for measuring 
temperatures using RTDs. 
"""

from gpiozero import MCP3208

adc = MCP3208()

RTD0 = MCP3208(channel=0, port=0, device=0) #ADC input 0 settings using MCP3208 hardware interface (MOSI: GPIO10, MISO: GPIO9, CLK: GPIO11, CS0: GPIO8)
RTD1 = MCP3208(channel=1, port=0, device=0) 
RTD2 = MCP3208(channel=2, port=0, device=0)
RTD3 = MCP3208(channel=3, port=0, device=0)
RTD4 = MCP3208(channel=4, port=0, device=0)
RTD5 = MCP3208(channel=5, port=0, device=0)
RTD6 = MCP3208(channel=6, port=0, device=0)
RTD7 = MCP3208(channel=7, port=0, device=0)

RTD_list = [RTD0, RTD1, RTD2, RTD3, RTD4, RTD5, RTD6, RTD7]

def TFD(data):
    """
    For converting single data points into temperature readings.
    Data should be a float value from 0 to 1, as read by ADC with Vref at 3.3 V
    Improved version of TFD with more accurate curve fitting and higher order interpolations
    (see Documentation for details, MATLAB files included)
    """
    a = 2.725076799546500*10**(-12)
    b = -1.231253679636238*10**(-8)
    c = 3.046224786805958*10**(-5)
    d = 0.221027985508455
    e = -241.9045208388455

    if data != 1:
        V = data * 3.3
        R = (2700 * V) / (3.3 - V)  # Based on voltage divider circuit with 2.7 k resistor
        # in series with RTD powered on 3.3 V,

    else:  # in case of dividing by 0
        R = 100000  # arbitrary value for extremely high R (most likely
        # due to error)

    T = a*R**4 + b*R**3 + c*R**2 + d*R + e
    return round(T, 2)


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
