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
pinDict = {'A0': 37,'A1': 36, 'A2': 23, 'A3': 22, 2: 32, 3: 18, 4: 28, 5: 17, 6: 24, 7: 27, 8: 26, 9: 19, 10: 7, 11: 25, 12: 38, 13: 39, 'ledGalileo': 3}
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
MAX_ANGLE = 247
MIN_POWER = 450000
MAX_POWER = 1000000
MIN_ENCODER = 2
MAX_ENCODER = 3685
CONST_ENCODER = 0.0
ERRO = 2
INTERVAL = 200
OFFSET_COM = 200
BAUD_RATE = 115200

# constantes do motor
MOTOR_FREE = 0
MOTOR_STOP = 1000000

# variaveis de controle
ser = serial.Serial("/dev/ttyGS0", 115200)
inString = ""
isHearing = 1
inputAngle = 0
lastPrintedAngle = 0
pinsSet = []
pinsSetPWM = []

# variaveis do digital input
selectAngle = [0, 0, 0, 0, 0]
selectMode = [0, 0, 0]

def setup():
    global CONST_ENCODER,inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

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

    CONST_ENCODER = MAX_ANGLE / float(MAX_ENCODER)
    # print "CONST_ENCODER: ", CONST_ENCODER

    # inputAngle = readStorageValue()
    # dbug("StorageValue: " + str(inputAngle))

def connect(text):
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    ser = serial.Serial(text, 115200)
    if not ser.isOpen():
        ser.open()
    if ser.isOpen:
        ser.flushInput()
        ser.write("a111c".encode())

    # print("endconnect")

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

def readStorageValue():
    value = 0
    try:
        with open("/home/root/storage", "r") as openFile:
            value = int(openFile.read())
            openFile.close()
    except IOError:
        print("IOError: Could not read storage")
    return value

def storegeValue(value):
    try:
        with open("/home/root/storage", "w") as openFile:
            openFile.write(value)
            openFile.close()
    except IOError:
        print("IOError: Could not write %d to storage" % value)

# begin -- motor functions

# Envia mensagem dentro do protocolo
def printAngle(angle):
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    if lastPrintedAngle != angle and isHearing:
        s = str(angle + OFFSET_COM)
        println("a" + s + "c")
        lastPrintedAngle = angle

def motorStop(): # power 0 to 255
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    digitalWrite(IN1, HIGH)
    digitalWrite(IN2, HIGH)
    analogWrite(EN, MOTOR_STOP) 

def motorFree(): # power 0 to 255
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    digitalWrite(IN1, LOW)
    digitalWrite(IN2, LOW)
    analogWrite(EN, MOTOR_FREE)

def goClockWise(distance): # power 0 to 255
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    power = motorMap(distance)
    digitalWrite(IN1, LOW)
    digitalWrite(IN2, HIGH)
    analogWrite(EN, power)
    ## delay(50)

def goCClockWise(distance): # power 0 to 255
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    power = motorMap(distance)
    digitalWrite(IN1, HIGH)
    digitalWrite(IN2, LOW)
    analogWrite(EN, power)
    ## delay(50)

def realPosition():
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    value = analogRead(ENCODER)
    angle = int(value * CONST_ENCODER)

    return angle

def realMeanPosition():
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    value = 0
    sampleSize = 10
    for i in xrange(0, sampleSize):
        value += int(analogRead(ENCODER))

    angle = int(value/float(sampleSize) * CONST_ENCODER)
    # dbug("-angle: " + str(angle))
    printAngle(angle)
    return angle

def goToDegree(degree):
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    if degree >= MIN_ANGLE and degree <= MAX_ANGLE:
        realPosition = realMeanPosition()

        if (realPosition < (degree - ERRO)) or (realPosition > (degree + ERRO)):
            dbug("-inputAngle: " + str(inputAngle))
            dbug("-realPosition: " + str(realPosition))
            distance = realPosition - degree
            if distance < 0:
                goClockWise(abs(distance))
            else:
                goCClockWise(distance)
            realPosition = realMeanPosition()
        else:
            motorStop()
            motorFree()

def pinsRead():
    global selectAngle, selectMode
    # dbug("-PinsRead Begin")
    selectMode[0] = digitalRead(SELECT)
    # selectMode[1] = digitalRead(NULL1)
    # selectMode[2] = digitalRead(NULL2)
    selectAngle[0] = digitalRead(LSB)
    selectAngle[1] = digitalRead(B2)
    selectAngle[2] = digitalRead(B3)
    selectAngle[3] = digitalRead(B4)
    selectAngle[4] = digitalRead(MSB)
    # dbug("M: " + str(selectMode[0]) + " LSB: " + str(selectAngle[0]) + " " + str(selectAngle[1]) + " " + str(selectAngle[2]) + " " + str(selectAngle[3]) + " " + str(selectAngle[4]))
def binTodegree():
    decimal = 0
    
    for i in xrange(0, 5):
        decimal += (int(selectAngle[i]) * (1<<i))
    
    degree = decimal * 7.74 # 360/31 = 11.61 or 240/31 = 7.74
    
    return degree

def newMap(x, in_min, in_max, out_min, out_max):
    ## http://forum.arduino.cc/index.php?topic=38006.0
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def motorMap(distance):
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    return newMap(distance, MIN_ANGLE, MAX_ANGLE, MIN_POWER, MAX_POWER)

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

    # print("printpinMode")

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

    # print("endpinModeDigital")

def pinModeAnalog(pin):
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    pinMode(pin, OUTPUT)
    digitalWrite(pin, 0)

    # print("endpinModeAnalog")

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

    # print("endpinModePWM")

def digitalRead(pin):
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    # gets the real pin
    pin = pinDict[pin]
    value = 0
    try:
        with open("/sys/class/gpio/gpio" + str(pin) + "/value", "r") as openFile:
            value = str(openFile.read())
            openFile.close()
    except IOError:
        print("IOError: could not read from GPIO %d" % pin)

    # print("enddigitalRead")

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

    # print("endigitalWrite")

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
            print("IOError: Could't read in_voltage, pin " + str(pin))

def analogWrite(pin, duty_cycle):
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode

    # gets the real pin
    pin = pinDict[pin]
    duty_cycle = int(duty_cycle)
    # dbug("echo " + str(duty_cycle) + " > /sys/class/pwm/pwmchip0/pwm" + str(pin) + "/duty_cycle" )
    try:
        with open("/sys/class/pwm/pwmchip0/pwm" + str(pin) + "/duty_cycle","w") as d:
            d.write(str(MAX_POWER if duty_cycle > MAX_POWER else duty_cycle))
            d.close()
    except IOError:
        print("IOError: could not set pwm duty_cycle to pin %d" % pin)

    # time.sleep(0.5)

def println(angle):
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode   
    if self.serial.isOpen():
        text = "a" + str(angle + 200) + "c\n"
        self.serial.write(text.encode())

def printl(angle):
    global inString, isHearing, inputAngle, lastPrintedAngle, pinsSet, pinsSetPWM, selectAngle, selectMode   
    if self.serial.isOpen():
        text = "a" + str(angle + 200) + "c"
        self.serial.write(text.encode())

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

def dbug(s):
    print s

if __name__ == "__main__":
    setup()
    while True:
        # if is some user input to read 
        if select.select([sys.stdin,],[],[],0.0)[0]:
            if sys.stdin.read(1) == 'e':
                quit()

        pinsRead();
        if int(selectMode[0]) == 0: # automatico
            goToDegree(binTodegree())
        else: # manual
            serialRead()
            goToDegree(inputAngle)
                
