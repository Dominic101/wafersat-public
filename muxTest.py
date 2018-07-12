import RPi.GPIO as GPIO
from mcp3208 import MCP3208
import time

muxLSB = 40
muxMSB = 38
chanSel = [muxMSB, muxLSB]

lowDivider = 11000
maxADC = 4096
vref = 1.024

adc = MCP3208()
GPIO.setmode(GPIO.BOARD)
GPIO.setup(chanSel, GPIO.OUT);
GPIO.output(chanSel, GPIO.LOW)

while(True):
    upper = adc.read(0)
    lower = adc.read(1)

    vUpper = upper/maxADC *vref
    vLower = lower/maxADC *vref

    print("First Mux Channel (0)")
    print("\t Upper Mux: " + str(vUpper))
    print("\t Lower Mux: " + str(vLower))

    GPIO.output(chanSel, (GPIO.LOW, GPIO.HIGH))
    time.sleep(3)

    upper = adc.read(0)
    lower = adc.read(1)

    vUpper = upper/maxADC *vref
    vLower = lower/maxADC *vref

    print("Second Mux Channel (1)")
    print("\t Upper Mux: " + str(vUpper))
    print("\t Lower Mux: " + str(vLower))

    GPIO.output(chanSel, (GPIO.HIGH, GPIO.LOW))
    time.sleep(3)
    
    upper = adc.read(0)
    lower = adc.read(1)

    vUpper = upper/maxADC *vref
    vLower = lower/maxADC *vref

    print("Third Mux Channel (2)")
    print("\t Upper Mux: " + str(vUpper))
    print("\t Lower Mux: " + str(vLower))

    GPIO.output(chanSel, GPIO.HIGH)
    time.sleep(3)
        
    upper = adc.read(0)
    lower = adc.read(1)

    vUpper = upper/maxADC *vref
    vLower = lower/maxADC *vref

    print("Last Mux Channel (3)")
    print("\t Upper Mux: " + str(vUpper))
    print("\t Lower Mux: " + str(vLower))

    GPIO.output(chanSel, GPIO.LOW)
    time.sleep(3)
