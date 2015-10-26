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
pinDict = {'A0': 37,'A1': 36, 'A2': 23, 'A3': 22, 2: 14, 3: 3, 4: 28, 5: 5, 6: 6, 7: 27, 8: 26, 9: 1, 10: 7, 11: 4, 12: 38, 13: 39, 'ledGalileo': 3}

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
  # gets the real pin
  pinsSet.append(pin)
  try:
    with open("/sys/class/gpio/export", "w") as openFile:
      openFile.write(str(pin))
  except IOError:
      print("INFO: GPIO %d already exists, skipping export", pin)

  openFile = open("/sys/class/gpio/gpio" + str(pin) + "/direction", "w")
  openFile.write(direction)
  openFile.close()

def pinModeDigital(pin, direction):
  pin = pinDict[pin]
  pinMode(pin, direction)
  print pin
  openFile = open("/sys/class/gpio/gpio" + str(pin) + "/drive", "w")
  openFile.write(DIGITALPINMODE)
  openFile.close()

def pinModeAnalog(pin):
  pin = pinDict[pin]
  pinMode(pin, OUTPUT)
  digitalWrite(pin, 0)

def pinModePWM(pin):
  pin = pinDict[pin]
  pinsSetPWM.append(pin)
  try:
    with open("/sys/class/pwm/pwmchip0/export","w") as export:
      export.write(str(pin))
  except IOError:
    print "IOError: could not export pwm"

  try:
    with open("/sys/class/pwm/pwmchip0/pwm" + str(pin) + "/enable","w") as enable:
      enable.write("1")
  except IOError:
    print "IOError: could not enable pwm"

  try:
    with open("/sys/class/pwm/pwmchip0/pwm" + str(pin) + "/period","w") as p:
      p.write(str(PERIOD))
  except IOError:
    print "IOError: could not set pwm period"

def digitalRead(pin):
  # gets the real pin
  pin = pinDict[pin]
  openFile = open("/sys/class/gpio/gpio" + str(pin) + "/value", "r")
  value = str(openFile.read())
  openFile.close()
  return value
  
def digitalWrite(pin, value):
  # gets the real pin
  pin = pinDict[pin]
  openFile = open("/sys/class/gpio/gpio" + str(pin) + "/value", "w")
  openFile.write(str(value))
  openFile.close()

def analogRead(pin):
  # gets the number of pin AX where X is a number 
  pin = pin[1]
  try:
    with open("/sys/bus/iio/devices/iio:device0/in_voltage" + str(pin) + "_raw", 'r') as openFile:
      return openFile.read()
  except IOError:
    print("IOError: Could't read in_voltage" + str(pin))

def analogWrite(pin, duty_cycle):
  # gets the real pin
  pin = pinDict[pin]
  try:
    with open("/sys/class/pwm/pwmchip0/pwm" + str(pin) + "/duty_cycle","w") as d:
      d.write(str(duty_cycle))
  except IOError:
    print "IOError: could not set pwm duty_cycle"

def println(s):
  openFile = open("/dev/ttyGS0", "w")
  openFile.write(s + "\n")
  openFile.close()

def printl(s):
  openFile = open("/dev/ttyGS0", "w")
  openFile.write(s)
  openFile.close()

def quit():
  unexport()
  exit(0)

def unexport():
  for pin in pinsSet:
    try:
      with open("/sys/class/gpio/unexport", "w") as openFile:
        openFile.write(str(pin))
    except IOError:
      print("INFO: GPIO %d dosen't exists, skipping unexport", pin)

  for pin in pinsSetPWM:
    try:
      with open("/sys/class/pwm/pwmchip0/pwm" + str(pin) + "/duty_cycle","w") as d:
        d.write(str(0))
    except IOError:
      print "IOError: could not set pwm duty_cycle"
    
    try:
      with open("/sys/class/pwm/pwmchip0/unexport","w") as openFile:
        openFile.write(str(pin))
    except IOError:
      print("INFO: PWM %d dosen't exists, skipping unexport", pin)


def main():
  # pinModeDigital(LED, OUTPUT)
  pinMode(PIN7, INPUT)
  pinModeAnalog(PINA0)
  pinModeDigital(LEDOFF, OUTPUT)
  pinModeDigital(PININ1, OUTPUT)
  pinModeDigital(PININ2, OUTPUT)
  pinModePWM(PINPWM)

  duty_cycle = 400000
  period = 1000000
  while True:
    print(pinsSet)
    print(pinsSetPWM)
    # if is some user input to read 
    if select.select([sys.stdin,],[],[],0.0)[0]:
      if sys.stdin.read(1) == 'e':
        quit()
    
    print("on")
    println("on")
    print("analogRead = " + analogRead(PINA0))
    print("digitalRead = " + digitalRead(PIN7))

    digitalWrite(PININ1, LOW)
    analogWrite(PINPWM, duty_cycle)

    digitalWrite(LED, HIGH)
    digitalWrite(LEDOFF, HIGH)
    time.sleep(1)
    
    print("off")
    println("off")
    
    print(digitalRead(PIN7))
    
    digitalWrite(LED, LOW)
    digitalWrite(LEDOFF, LOW)

    time.sleep(1)

    duty_cycle = (duty_cycle + 100000)
    if duty_cycle > 1000000:
      duty_cycle = 400000

main() 