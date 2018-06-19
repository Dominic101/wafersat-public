""" Author: Charles Lindsay and Phil Murzynowski 2.01.18 For WaferSat 
UROP Thermal Test Circuit with PCBSat Rev1 """

import RPi.GPIO as IO          #calling header file which helps us use GPIO’s of PI
import time                    #calling time to provide delays in program
from gpiozero import LED, MCP3208
led = LED(21) #initializes status LED connected to GPIO21

led.on()
sleep(10)
#x = 10
##heater 1: GPIO 25
##heater 2: GPIO 24
#
class heaters:

 def __init__(self,GPIO,freq=20):
  '''
  #freq is input 
  '''
  IO.setwarnings(False)          #do not show any warnings
  IO.setmode(IO.BCM)             #we are programming the GPIO by BCM pin numbers. (PIN35 as ‘GPIO19’)
  
  IO.setup(GPIO,IO.OUT)          #initialize GPIO as an output
  
  self.heater = IO.PWM(GPIO,freq)  #GPIO as PWM output, with frequency as freq
  
  
  self.heater.start(0)                     #generate PWM signal with 0% duty cycle
  
 def set_DC(self, newDC):             #change duty cycle of heater
  '''
#  heater should be an integer representing the heater number
#  newDC should be an integer from 0-100 representing % duty cycle
  '''

  self.heater.ChangeDutyCycle(newDC)

 def kill_heater(self):

  self.heater.stop()
  

heater2 = heaters(24)
print("heaters started")
#heater2.set_DC(x)
#print("heater 2 at ", x)

#heater2.kill_heater()
#print("heater stopped")


IO.cleanup()