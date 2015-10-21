# -*- coding: utf-8 -*-
from __future__ import unicode_literals
#!/usr/bin/python
import sys
import time

def pinMode(pin, direction):
  try:
    gpio = open("/sys/class/gpio/export","w")
    gpio.write("$pin")
    gpio.close()
  except IOError:
    print "INFO: GPIO $pin already exists, skipping export"
  gpio = open( "/sys/class/gpio/gpiopin/direction", "w" )
  gpio.write(direction)
  gpio.close()

def digitalWrite(pin, value):
  gpio = open( "/sys/class/gpio/gpio$pin/value", "w" )
  gpio.write( str( value ) )
  gpio.close()

pinMode(3, "out")
while True:
  print "on"
  digitalWrite(3, 1)
  time.sleep( 1 )
  print "off"
  digitalWrite(3, 0)
  time.sleep( 1 ) 