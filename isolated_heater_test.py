#!/usr/bin/python
#Created by Charles Lindsay with contributions from Phil Murzynowski

import sh
from time import sleep
import time
import subprocess
from datetime import datetime
import csv
import RTD_V2 as rtd 
import os
from pwm_V2 import heaters

def update_tempList(tempList): 
	#add new rtd reading to end of list, delete 1st element (oldest data point)

	for i in range(0,8):
		tempList[i].append(rtd.get_temp(i))
		del tempList[i][0]	
	
	return tempList

def average_temp(tempList):
	#returns list contained average rtd temperatures
	avgTemps = []
	
	for i in range(0,8):
		sumd = 0
		average = 0
		for j in range(0,len(tempList[i])):
			sumd+=tempList[i][j]
		average=sumd/len(tempList[i])
		avgTemps.append(average)
	
		       
	return avgTemps

			       
def highest_deriv(To,T1): #MODIFIED FOR ISOLATED HEATER TEST
	#calculates dT/dt for each rtd, and returns one with highest slope
	dTdt = 0
	rates = []
	for i in range(0,len(To)):
		rates.append((T1[i]-To[i])/dt)
			       

	max = rates[0]		  
	for i in range(0,len(rates)-1):
		if rates[i+1] > rates[i]:
			max = rates[i+1]
			
	return max
	#return rates[0] #for isolated heater uncomment and comment out return max

def record_temp_filtered(tempList):
	
	timeStamp = time.time()
	l = average_temp(tempList)
	rtd0 = l[0]
	rtd1 = l[1]
	rtd2 = l[2]
	rtd3 = l[3]
	rtd4 = l[4]
	rtd5 = l[5]
	rtd6 = l[6]
	rtd7 = l[7]

	Writer.writerow([timeStamp, rtd0, rtd1,rtd2,rtd3,rtd4,rtd5,rtd6,rtd7])
	csvfile.flush()
	
def record_temp_raw():
 		
	#writes time and temp to csv
	timeStamp = time.time()
	
	rtd0_temp = rtd.get_temp(0)
	rtd1_temp = rtd.get_temp(1)
	rtd2_temp = rtd.get_temp(2)
	rtd3_temp = rtd.get_temp(3)
	rtd4_temp = rtd.get_temp(4)
	rtd5_temp = rtd.get_temp(5)
	rtd6_temp = rtd.get_temp(6)
	rtd7_temp = rtd.get_temp(7)
     				
	Writer.writerow([timeStamp, rtd0_temp, rtd1_temp, rtd2_temp, rtd3_temp, rtd4_temp, rtd5_temp, rtd6_temp, rtd7_temp])
	csvfile.flush()
	
'''Start program----------------------------------
'''

testDC = 0
dutyCycles = [10,20,30] #list as integers, %DC

heater1 = heaters(25,20) #init heater 1 at freq = 20 Hz
heater2 = heaters(24,20)

for i in range(0,len(dutyCycles)):
        
	testDC = dutyCycles[i]

	# Create file
	tim = datetime.now()
	tim = tim.strftime("%Y-%m-%d %H:%M")
	filename = '/home/pi/Desktop/Isolated_Heater_Test_Data/heater_test_DC' + str(testDC) + '_'  + tim + '.csv'
	filename = filename.replace(' ', '')

	print('Filename for generated file will be:')
	print(filename)
	print('Starting test with ' + str(testDC) + '% duty cycle in 2 seconds')

	sleep(2)
      
        #create new csv file to hold RTD and time data
	
	with open(filename, 'w', newline='') as csvfile: 

		Writer = csv.writer(csvfile, delimiter=',')
		Writer.writerow(['time','RTD0','RTD1','RTD2','RTD3','RTD4','RTD5','RTD6','RTD7'])
		csvfile.flush()
    		
		#initialize vars/lists	
	       
		tempList = [[],[],[],[],[],[],[],[]]
		freq = 4 #data recording freq in Hertz
		dt = 0.25 #time between average temp samples taken
		avgTo = []
		avgT1 = []
		#intialize RTD temperature lists with 20 temp values
		for i in range(0,300):
			for j in range(0,8):
				tempList[j].append(rtd.get_temp(j))	
		
		avgTo = average_temp(tempList)		
		
		#collect RTD and time, store in csv and repeat until steady state high then low reached
		for i in range(0,2):
			if i == 0:
				heater1.set_DC(testDC)
				heater2.set_DC(testDC)
				print("Heaters on, Starting Data Collection")
			else:
				heater1.set_DC(0)
				heater2.set_DC(0)
				print("Heaters off, Continuing Test")
			
			dTdt = 1
			ntime = time.time() + 1/freq  #setup 1st time to take raw value
			ncutofftime = time.time() + 3*dt 
			cutoff = 0
			c = 0
			#arrayfiltered = []
			while cutoff != 1:
			#count = 0 #temp var for test prep
			#while count < 31: #temp loop for test prep
				tempList = update_tempList(tempList)

				ctime = time.time()

				if abs(ctime - ncutofftime) <= 0.005:
					#count+=1 #temp var for test prep
					ncutofftime+=dt
					avgT1 = average_temp(tempList)
					dTdt = highest_deriv(avgTo,avgT1)
					#print(avgT1[0])
					print(avgT1[0])
					#arrayfiltered.append(avgT1[0])
					avgTo = average_temp(tempList)
					
					'''for e in avgTo: #check to see if any RTD temp > 60 deg
						if e > 60:
							dTdt = 0
							print("Temp > 60 deg C reached. Stopping test")
					'''
				#records temp at set freq
				if abs(ctime - ntime) <= 0.005: #if current time approximately equals next time where sample should be taken
					ntime+=1/freq
					record_temp_filtered(tempList)
		
				
				if abs(dTdt) < 0.005:
					if c == 0:	
						print("steady state reached, holding for 30 s")
						sstime = time.time()
						c = 1
					else:
						if time.time()-sstime >=30:
							cutoff = 1

		print("End of Test DC = ", testDC)

heater1.kill_heater()
heater2.kill_heater()
print("All ", len(dutyCycles)," Tests Completed :)")
