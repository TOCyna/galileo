# -*- coding: utf-8 -*-
from __future__ import unicode_literals
#!/usr/bin/python
import sys
import time
import select

# constants
LOW = 0
HIGH = 1
OUTPUT = "out"
INPUT = "in"
PERIOD = 1000000 #PWM Period
# pinmap arduino -> galileo
pinDict = {'A0': 37,'A1': 36, 'A2': 23, 'A3': 22, 2: 14, 3: 3, 4: 28, 5: 5, 6: 6, 7: 27, 8: 26, 9: 1, 10: 7, 11: 25, 12: 38, 13: 39, 'ledGalileo': 3}

# When configured for output GPIO ports that are connected to CY8C9520A can be configured to one of the following drive modes:
  # Resistive high, strong low (drive = pullup)
  #   This is the default, but it not suitable for driving devices that source significant current, for example for driving an LED connected between GPIO port and GND (it will work though if the LED is connected between GPIO and 5V or 3.3V rails)
  # Resistive low, strong high (drive = pulldown)
  #   Strong low and high (drive = strong)
  # This mode is appropriate for most applications.
  #   High Z state (drive = hiz)
  # (CY8C9520A also supports open drain and open source drive modes, but it is not currently exposed through SysFS)
DIGITALPINMODE = "strong"

# pins
LED = 'ledGalileo'
LEDOFF = 13
PIN7 = 7
PINA0 = 'A0'
PININ1 = 11
PININ2 = 12
PINPWM = 10
pinsSet = []
pinsSetPWM = []

# functions
def pinMode(pin, direction):
    # global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    # gets the real pin
    pin = pinDict[pin]
    pinsSet.append(pin)
    try:
        with open("/sys/class/gpio/export", "w") as openFile:
            openFile.write(str(pin))
            openFile.close()
    except IOError:
        print("INFO: GPIO %d already exists, skipping export" % pin)

    try:
        with open("/sys/class/gpio/gpio" + str(pin) + "/direction", "w") as openFile:
            openFile.write(direction)
            openFile.close()
    except IOError:
        print("INFO: Can't set direction in GPIO %d" % pin)

    print("printpinMode")

def pinModeDigital(pin, direction):
    # global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    pinMode(pin, direction)
    pin = pinDict[pin]

    try:
        with open("/sys/class/gpio/gpio" + str(pin) + "/drive", "w") as openFile:
            openFile.write(DIGITALPINMODE)
            openFile.close()
    except IOError:
        print("INFO: Can't set drive mode in GPIO %d" % pin)

    print("endpinModeDigital")

def pinModeAnalog(pin):
    # global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    pinMode(pin, OUTPUT)
    digitalWrite(pin, 0)

    print("endpinModeAnalog")

def pinModePWM(pin):
    # global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    pin = pinDict[pin]
    pinsSetPWM.append(pin)
    try:
        with open("/sys/class/pwm/pwmchip0/export","w") as export:
            export.write(str(pin))
            export.close()
    except IOError:
        print("IOError: could not export pwm %d" % pin)

    try:
        with open("/sys/class/pwm/pwmchip0/pwm" + str(pin) + "/enable","w") as enable:
            enable.write("1")
            enable.close()
    except IOError:
        print("IOError: could not enable pwm %d" % pin)

    try:
        with open("/sys/class/pwm/pwmchip0/pwm" + str(pin) + "/period","w") as p:
            p.write(str(PERIOD))
            p.close()
    except IOError:
        print("IOError: could not set pwm period %d" % pin)

    print("endpinModePWM")

def digitalRead(pin):
    # global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    # gets the real pin
    pin = pinDict[pin]
    value = 0
    try:
        with open("/sys/class/gpio/gpio" + str(pin) + "/value", "r") as openFile:
            value = str(openFile.read())
            openFile.close()
    except IOError:
        print("IOError: could not read from GPIO %d" % pin)

    print("enddigitalRead")

    return value
  
def digitalWrite(pin, value):
    # global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    # gets the real pin
    pin = pinDict[pin]
    try:
        with open("/sys/class/gpio/gpio" + str(pin) + "/value", "w") as openFile:
            openFile.write(str(value))
            openFile.close()
    except IOError:
        print("IOError: could not write value to GPIO %d" % pin)

def analogRead(pin):
    # global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    # gets the number of pin AX where X is a number 
    pin = pin[1]
    try:
        with open("/sys/bus/iio/devices/iio:device0/in_voltage" + str(pin) + "_raw", 'r') as openFile:
            value = openFile.read()
            openFile.close()
        return value
    except IOError:
            print("IOError: Could't read in_voltage" + str(pin))

def analogWrite(pin, duty_cycle):
    # global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    # gets the real pin
    pin = pinDict[pin]
    try:
        with open("/sys/class/pwm/pwmchip0/pwm" + str(pin) + "/duty_cycle","w") as d:
            d.write(str(duty_cycle))
            d.close()
    except IOError:
        print("IOError: could not set pwm duty_cycle")

def newMap(x, in_min, in_max, out_min, out_max):
    ## http://forum.arduino.cc/index.php?topic=38006.0
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def println(s):
    # global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode
    print(s+"\n")
    try:
        with open("/dev/ttyGS0", "w") as openFile:
            openFile.write(s + "\n")
            openFile.close()
    except IOError:
        print("IOError: could not write to /dev/ttyGS0")

def printl(s):
    # global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode
    print(s)

    try:
        with open("/dev/ttyGS0", "w") as openFile:
            openFile.write(s)
            openFile.close()
    except IOError:
        print("IOError: could not write to /dev/ttyGS0")

# end -- functions "arduino"

# begin -- other functions

def quit():
    unexport()
    exit(0)

def unexport():
    # global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    for pin in pinsSet:
        try:
            with open("/sys/class/gpio/unexport", "w") as openFile:
                openFile.write(str(pin))
                openFile.close()
        except IOError:
            print("INFO: GPIO %d dosen't exists, skipping unexport" % pin)

    for pin in pinsSetPWM:
        try:
            with open("/sys/class/pwm/pwmchip0/pwm" + str(pin) + "/duty_cycle","w") as d:
                d.write(str(0))
                d.close()
        except IOError:
            print "IOError: could not set pwm duty_cycle"
    
        try:
            with open("/sys/class/pwm/pwmchip0/unexport","w") as openFile:
                openFile.write(str(pin))
                openFile.close()
        except IOError:
            print("INFO: PWM %d dosen't exists, skipping unexport" % pin)

def main():
    pinModeDigital(LED, OUTPUT)
    pinMode(PIN7, INPUT)
    pinModeAnalog(PINA0)
    pinModeDigital(LEDOFF, OUTPUT)
    pinModeDigital(PININ1, OUTPUT)
    pinModeDigital(PININ2, OUTPUT)
    pinModePWM(PINPWM)

    duty_cycle = 200000
    period = 1000000
    while True:
        # print(pinsSet)
        # print(pinsSetPWM)
        # if is some user input to read 
        if select.select([sys.stdin,],[],[],0.0)[0]:
            if sys.stdin.read(1) == 'e':
                quit()
    
        # print("on")
        # println("on")
        value = 0
        for i in xrange(1,10):
            value += int(analogRead(PINA0))
        value /= 10
        # value = abs(value + 220)
        print("analogRead = " + str(value))
        print("angle = " + str(newMap(value, 2, 3685, 0, 247)))
        # print("digitalRead = " + digitalRead(PIN7))

        digitalWrite(PININ1, LOW)
        digitalWrite(PININ2, HIGH)
        analogWrite(PINPWM, duty_cycle)

        digitalWrite(LED, HIGH)
        digitalWrite(LEDOFF, HIGH)
        # time.sleep(1)
        
        # print("off")
        # println("off")
        
        # print(digitalRead(PIN7))
        
        digitalWrite(LED, LOW)
        digitalWrite(LEDOFF, LOW)

        time.sleep(1)

        duty_cycle = (duty_cycle + 100000)
        if duty_cycle > 1000000:
            duty_cycle = 400000

main() 