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
pinsSet = []
pinsSetPWM = []
# When configured for output GPIO ports that are connected to CY8C9520A can be configured to one of the following drive modes:
  # Resistive high, strong low (drive = pullup)
  #   This is the default, but it not suitable for driving devices that source significant current, for example for driving an LED connected between GPIO port and GND (it will work though if the LED is connected between GPIO and 5V or 3.3V rails)
  # Resistive low, strong high (drive = pulldown)
  #   Strong low and high (drive = strong)
  # This mode is appropriate for most applications.
  #   High Z state (drive = hiz)
  # (CY8C9520A also supports open drain and open source drive modes, but it is not currently exposed through SysFS)
DIGITALPINMODE = "strong"

# alocacao dos pinos
EN = 10 # pino de enable do L293d
IN1 = 11 # pino A1 de direcao do L293D
IN2 = 12 # pino A2 de direcao do L293D
ENCODER = A0
MSB = 9
B4 = 8
B3 = 7
B2 = 6
LSB = 5
NULL2 = 4
NULL1 = 3
SELECT = 2
LED = 13

# constantes globais
MIN_ANGLE = 0
MAX_ANGLE = 240
MIN_POWER = 60
MAX_POWER = 255
MIN_ENCODER = 0
MAX_ENCODER = 1022
CONST_ENCODER
ERRO = 0
INTERVAL = 200
OFFSET_COM = 200

# constantes do motor
MOTOR_FREE = 0
MOTOR_STOP = 220

# variaveis de controle
inString = ""
isHearing = 0
input = 0
com = 0
lastPrintedAngle = 0

# variaveis do digital input
selectAngle[5]
selectMode[3]

def setup():
    # Serial.begin(9600)

    pinModePWM(EN) # PWM    
    # PonteH
    pinMode(IN1, OUTPUT) # right gate L293d
    pinMode(IN2, OUTPUT) # left gate L293

    # switch
    pinMode(SELECT, INPUT
    pinMode(NULL1, INPUT)
    pinMode(NULL2, INPUT)
    pinMode(LSB, INPUT)
    pinMode(B2, INPUT)
    pinMode(B3, INPUT)
    pinMode(B4, INPUT)
    pinMode(MSB, INPUT)

    pinModeAnalog(ENCODER) 
    pinMode(LED, OUTPUT)

    # CONST_ENCODER = (float)MAX_ANGLE / (float)MAX_ENCODER //debug

# def serialRead():
#   if (Serial.available()) {
#     int inChar = Serial.read()
#     char a = (char)inChar
#     if (a == 'a') { //Inicio de mensagem
#       inString = ""
#       com = 0
#     }
#     if (isDigit(inChar)) { //Mensagem
#       inString += (char)inChar
#     }
#     if (inChar == 'c') { //Fim de mensagem
#       com = inString.toInt()
#       if (com == 111) {
#         isHearing = 1 //Habilita o envio Serial (Handshack)
#         Serial.println("a111c")
#         Serial.flush()
#         digitalWrite(LED, HIGH)
#       } else if (com == 101) {
#         isHearing = 0 //Desabilita o envio Serial (CloseConection)
#         digitalWrite(LED, LOW)
#       } else if (com == 100) {
#         realMeanPosition()
#       } else {
#         com = com - OFFSET_COM
#         if (com >= MIN_ANGLE and com <= MAX_ANGLE) {
#           input = com
#         }
#       }
#       com = 0 //Limpa para receber proxima mensagem
#       inString = "" //Limpa para receber proxima mensagem
#     }
#   }
# }

# begin -- motor functions

# Envia mensagem dentro do protocolo
def printAngle(int angle):
    if lastPrintedAngle != angle and isHearing:
        s = str(angle + OFFSET_COM)
        # Serial.println('a' + s + 'c')
        lastPrintedAngle = angle

def motorStop(): # power 0 to 255
    analogWrite(EN, MOTOR_STOP) 
    digitalWrite(IN1, HIGH)
    digitalWrite(IN2, HIGH)

def motorFree(): # power 0 to 255
    analogWrite(EN, MOTOR_FREE)
    digitalWrite(IN1, LOW)
    digitalWrite(IN2, LOW)

def goClockWise(distance): # power 0 to 255
    power = motorMap(distance)
    ## Serial.println("C: " + String(power))
    analogWrite(EN, power)
    digitalWrite(IN1, LOW)
    digitalWrite(IN2, HIGH)
    ## delay(50)

def goCClockWise(distance): # power 0 to 255
    power = motorMap(distance)
    ## Serial.println("CC: " + String(power))
    analogWrite(EN, power)
    digitalWrite(IN1, HIGH)
    digitalWrite(IN2, LOW)
    ## delay(50)

def realPosition():
    value = analogRead(ENCODER)
    ## int degree = map (value,0,1023,0, 360)
    return value

def realMeanPosition():
    value = 0
    sampleSize = 10
    for i in xrange(0, sampleSize):
        value += analogRead(ENCODER)
    angle = int((value/sampleSize) * CONST_ENCODER)
    printAngle(angle)
    return angle

def goToDegree(degree):
    if degree >= MIN_ANGLE and degree <= MAX_ANGLE:
        realPosition = realMeanPosition()

        if (realPosition < (degree - ERRO)) or (realPosition > (degree + ERRO)):
            distance = realPosition - degree
                if distance < 0:
                    goClockWise(abs(distance))
                else:
                    goCClockWise(distance)
            realPosition = realMeanPosition()

    if realPosition == degree:
        motorStop()

def newMap(x, in_min, in_max, out_min, out_max):
    ## http://forum.arduino.cc/index.php?topic=38006.0
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def motorMap(distance):
    return map(distance, MIN_ANGLE, MAX_ANGLE, MIN_POWER, MAX_POWER)

# end -- motor functions


# begin -- functions "arduino"

def pinMode(pin, direction):
    # gets the real pin
    pin = pinDict[pin]
    pinsSet.append(pin)
    try:
        with open("/sys/class/gpio/export", "w") as openFile:
            openFile.write(str(pin))
    except IOError:
        print("INFO: GPIO %d already exists, skipping export", pin)

    gpioFolder = "/sys/class/gpio/gpio" + str(pin)

    openFile = open(gpioFolder + "/direction", "w")
    openFile.write(direction)
    openFile.close()

    openFile = open(gpioFolder + "/drive", "w")
    openFile.write(DIGITALPINMODE)
    openFile.close()

def pinModeAnalog(pin):
    pinMode(pin, INPUT)
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

# end -- functions "arduino"

# begin -- other functions

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

# end -- other "arduino"

if __name__ == "__main__":
    setup()
    while True:
        # if is some user input to read 
        if select.select([sys.stdin,],[],[],0.0)[0]:
            if sys.stdin.read(1) == 'e':
                quit()
                
        serialRead()
        goToDegree(input)