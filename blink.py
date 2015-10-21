# -*- coding: utf-8 -*-
from __future__ import unicode_literals
#!/usr/bin/python
import sys
import time

def pins_export():
    try:
        pin1export = open("/sys/class/gpio/export","w")
        pin1export.write("3")
        pin1export.close()
    except IOError:
        print "INFO: GPIO 3 already exists, skipping export"

    fp1 = open( "/sys/class/gpio/gpio3/direction", "w" )
    fp1.write( "out" )
    fp1.close()

def write_led( value ):
    fp2 = open( "/sys/class/gpio/gpio3/value", "w" )
    fp2.write( str( value ) )
    fp2.close()


pins_export()
while True:
    print "on"
    write_led( 1 )
    time.sleep( 1 )
    print "off"
    write_led( 0 )
    time.sleep( 1 ) 