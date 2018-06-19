#!/usr/bin/python

# Last updated 2/25

# Program to display cpu load,  cpu temp, gpu temp, and rtd temps 
# measure temperature of ram?
# prints output, saves output to file whose name is based on time generated
#   next step is to export file to another computer
#   current method is to run below command in other persons terminal in the
# directory he/she wants to save it
#   scp pi@18.111.18.58:/home/pi/WaferSat/gpu_cpu_tempwhatevertimegoeshere.csv .
# or use google drive, ~20 sec delay
# -------------------------------------------------------------------------
# program some failsafes, maybe if it hits a certain temp stop with a margin so
# it is not on the edge of breaking
# see if it overheats outside of the vacuum temp
# use stress command, for ex:
#   stress -c 1 -q &
# then quit stress with
#   killall stress
# pi3 has four cores so above will overload one, cpu usage will be 25%
# but also need more fine-tuned setup, 25% steps are not precise enough to
# really test the pi

import sh
from time import sleep
import subprocess
from datetime import datetime
import csv
import newRTD_V2 as rtd 
import os

# filename formatting
time = datetime.now()

time = time.strftime("%Y-%m-%d %H:%M")
filename = 'pi_stats' + time + '.csv'
filename = filename.replace(' ', '')

# Pause here and print name so that it can be entered in auto_download script
print('Filename for generated file will be:')
print(filename)
print('Run auto_download now with filename for automatic download')
input('Press [ENTER] to contunue')

with open(filename, 'w', newline='') as csvfile:

    Writer = csv.writer(csvfile, delimiter=',')
    Writer.writerow(['CPU usage %', 'GPU temperature', 'CPU temperature'])
    csvfile.flush()

    while True:

        # measure temperatures
        cpu_temp = str(float(sh.cat('/sys/class/thermal/thermal_zone0/temp')) / 1000)
        gpu_temp = sh.vcgencmd('measure_temp')
        rtd0_temp = rtd.get_temp(0)
        rtd1_temp = rtd.get_temp(1)
        rtd2_temp = rtd.get_temp(2)
        rtd3_temp = rtd.get_temp(3)
        rtd4_temp = rtd.get_temp(4)
        rtd5_temp = rtd.get_temp(5)
        rtd6_temp = rtd.get_temp(6)
        rtd7_temp = rtd.get_temp(7)

        # formatting for temperature
        gpu_temp = gpu_temp.replace("temp=", "")
        gpu_temp = gpu_temp.replace("\'C", "   \'C")
        cpu_temp = cpu_temp + (6 - len(cpu_temp)) * ' '
        cpu_usage = 0 #FIX
        #cpu_usage = os.popen("top -n2 | awk '/Cpu\(s\):/ {print $2}'").readline(600).strip()  #INSERT Command here to get usage stat       
                
        print('-------------------')

        # measure cpu usage
        #subprocess.call(['./cpu_usage.sh'])
        #cpu_usage = str(subprocess.check_output(['./cpu_usage.sh']))
        #cpu_usage = cpu_usage.replace("b\'", "").replace("\\n\'", "")
        #subprocess.call('top -n2 | grep "%Cpu"')

        print('CPU Usage: ', cpu_usage)

        print('')
        print('GPU temp: ', gpu_temp)
        print('CPU temp: ', cpu_temp, "\'C")
        print('RTD0 temp: ', rtd0_temp)
        print('RTD1 temp: ', rtd1_temp)
        print('RTD2 temp: ', rtd2_temp)
        print('RTD3 temp: ', rtd3_temp)
        print('RTD4 temp: ', rtd4_temp)
        print('RTD5 temp: ', rtd5_temp)
        print('RTD6 temp: ', rtd6_temp)
        print('RTD7 temp: ', rtd7_temp)

        Writer.writerow([cpu_usage, gpu_temp, cpu_temp, rtd0_temp, rtd1_temp, rtd2_temp, rtd3_temp, rtd4_temp, rtd5_temp, rtd6_temp, rtd7_temp])
        csvfile.flush()

        sleep(1)
