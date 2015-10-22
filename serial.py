# -*- coding: utf-8 -*-
from __future__ import unicode_literals
#!/usr/bin/python
import sys
import time
import serial

def println(s):
  openFile = open("/dev/ttyGS0", "w")
  openFile.write(s + "\n")
  openFile.close()

def printl(s):
  openFile = open("/dev/ttyGS0", "w")
  openFile.write(s)
  openFile.close()

def read():
  openFile = open("/dev/ttyS0", "r")
  s = openFile.read()
  openFile.close()
  return s

def quit():
  exit(0)

def main():
  while True:
    s = read()
    print s

main() 