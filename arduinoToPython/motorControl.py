# -*- coding: utf-8 -*-
from __future__ import unicode_literals
#!/usr/bin/python
import sys
import time
import select
import serial

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

# alocacao dos pinos
EN = 10 # pino de enable do L293d
IN1 = 11 # pino A1 de direcao do L293D
IN2 = 12 # pino A2 de direcao do L293D
ENCODER = 'A0'
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
CONST_ENCODER = 0.0
ERRO = 0
INTERVAL = 200
OFFSET_COM = 200
BAUD_RATE = 115200

# constantes do motor
MOTOR_FREE = 0
MOTOR_STOP = 220

# variaveis de controle
ser = serial.Serial("/dev/ttyGS0", 115200)
inString = ""
isHearing = 0
inputAngle = 0
lastPrintedAngle = 0
pinsSet = []
pinsSetPWM = []

# variaveis do digital input
selectAngle = [0, 0, 0, 0, 0]
selectMode = [0, 0, 0]

def setup():
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    # connect("/dev/ttyGS0")

    pinModePWM(EN) # PWM    
    # PonteH
    pinMode(IN1, OUTPUT) # right gate L293d
    pinMode(IN2, OUTPUT) # left gate L293

    # switch
    pinMode(SELECT, INPUT)
    pinMode(NULL1, INPUT)
    pinMode(NULL2, INPUT)
    pinMode(LSB, INPUT)
    pinMode(B2, INPUT)
    pinMode(B3, INPUT)
    pinMode(B4, INPUT)
    pinMode(MSB, INPUT)

    pinModeAnalog(ENCODER) 
    pinMode(LED, OUTPUT)

    CONST_ENCODER = MAX_ANGLE / MAX_ENCODER

def connect(text):
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    ser = serial.Serial(text, 115200)
    if not ser.isOpen():
        ser.open()
    if ser.isOpen:
        ser.flushInput()
        ser.write("a111c".encode())

def serialRead():
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    if ser.isOpen():
        if ser.inWaiting():
            inChar = ser.read(1)
            if inChar:
                inChar = char.decode()
            if inChar == 'a': # Inicio de mensagem
                inString = ""
                com = 0
            elif inChar.isdigit(): # Mensagem
                inString += inChar
            if inChar == 'c': # Fim de mensagem
                com = int(inString)
                if com == 111:
                    isHearing = 1 # Habilita o envio Serial (Handshack)
                    println("a111c")
                    ser.flushInput()
                    digitalWrite(LED, HIGH)
                elif com == 101:
                    isHearing = 0 # Desabilita o envio Serial (CloseConection)
                    digitalWrite(LED, LOW)
                elif com == 100:
                    realMeanPosition()
                else:
                    com = com - OFFSET_COM
                    if com >= MIN_ANGLE and com <= MAX_ANGLE:
                        inputAngle = com
                com = 0 # Limpa para receber proxima mensagem
                inString = "" # Limpa para receber proxima mensagem

# begin -- motor functions

# Envia mensagem dentro do protocolo
def printAngle(angle):
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    if lastPrintedAngle != angle and isHearing:
        s = str(angle + OFFSET_COM)
        # Serial.println('a' + s + 'c')
        lastPrintedAngle = angle

def motorStop(): # power 0 to 255
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    analogWrite(EN, MOTOR_STOP) 
    digitalWrite(IN1, HIGH)
    digitalWrite(IN2, HIGH)

def motorFree(): # power 0 to 255
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    analogWrite(EN, MOTOR_FREE)
    digitalWrite(IN1, LOW)
    digitalWrite(IN2, LOW)

def goClockWise(distance): # power 0 to 255
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    power = motorMap(distance)
    ## Serial.println("C: " + String(power))
    analogWrite(EN, power)
    digitalWrite(IN1, LOW)
    digitalWrite(IN2, HIGH)
    ## delay(50)

def goCClockWise(distance): # power 0 to 255
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    power = motorMap(distance)
    ## Serial.println("CC: " + String(power))
    analogWrite(EN, power)
    digitalWrite(IN1, HIGH)
    digitalWrite(IN2, LOW)
    ## delay(50)

def realPosition():
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    value = analogRead(ENCODER)
    ## int degree = map (value,0,1023,0, 360)
    return value

def realMeanPosition():
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    value = 0
    sampleSize = 10
    for i in xrange(0, sampleSize):
        value += float(analogRead(ENCODER))
    angle = int((value/sampleSize) * CONST_ENCODER)
    printAngle(angle)
    return angle

def goToDegree(degree):
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

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
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    return map(distance, MIN_ANGLE, MAX_ANGLE, MIN_POWER, MAX_POWER)

# end -- motor functions


# begin -- functions "arduino"

def pinMode(pin, direction):
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

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

def pinModeDigital(pin, direction):
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    pinMode(pin, direction)
    pin = pinDict[pin]

    try:
        with open("/sys/class/gpio/gpio" + str(pin) + "/drive", "w") as openFile:
            openFile.write(DIGITALPINMODE)
            openFile.close()
    except IOError:
        print("INFO: Can't set drive mode in GPIO %d" % pin)

def pinModeAnalog(pin):
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    pinMode(pin, OUTPUT)
    digitalWrite(pin, 0)

def pinModePWM(pin):
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

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

def digitalRead(pin):
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    # gets the real pin
    pin = pinDict[pin]
    try:
        with open("/sys/class/gpio/gpio" + str(pin) + "/value", "r") as openFile:
            openFile = str(openFile.read())
            openFile.close()
    except IOError:
        print("IOError: could not read from GPIO %d" % pin)

    return value
  
def digitalWrite(pin, value):
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    # gets the real pin
    pin = pinDict[pin]
    try:
        with open("/sys/class/gpio/gpio" + str(pin) + "/value", "w") as openFile:
            openFile.write(str(value))
            openFile.close()
    except IOError:
        print("IOError: could not write value to GPIO %d" % pin)

def analogRead(pin):
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

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
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    # gets the real pin
    pin = pinDict[pin]
    try:
        with open("/sys/class/pwm/pwmchip0/pwm" + str(pin) + "/duty_cycle","w") as d:
            d.write(str(duty_cycle))
            d.close()
    except IOError:
        print("IOError: could not set pwm duty_cycle")

def println(s):
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode
    print(s+"\n")
    try:
        with open("/dev/ttyGS0", "w") as openFile:
            openFile.write(s + "\n")
            openFile.close()
    except IOError:
        print("IOError: could not write to /dev/ttyGS0")

def printl(s):
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode
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
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

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

# end -- other "arduino"

if __name__ == "__main__":
    setup()
    while True:
        # if is some user input to read 
        if select.select([sys.stdin,],[],[],0.0)[0]:
            if sys.stdin.read(1) == 'e':
                quit()
                
        serialRead()
        goToDegree(inputAngle)