#!/usr/bin/env python

""" 
  Core code for Hexapod         
  Copyright (c)  2015 Andrew Creahan.  All rights reserved.

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software Foundation,
  Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
"""

import sys, time, os
import serial     
import config
import math
from binascii import b2a_hex

atrib = config.atrib

RESET  = 0
WALK = 10 
ROTATE = 20
TRANSLATE = 30
SINLEG = 40

vals = [0,0,0,0,0]
x = 1

#define BALANCE     0x01
#define DOUBLE_T    0x02 //Double travel
#define STAND         0x04 //Sit Stand


start_time = time.time()
movecount = 0
move_time = 0 


class Driver:
    """ Class to open a serial port """
    def __init__(self,commandtypes, addtypes, port="/dev/ttyArb",baud=38400, interpolation=False, direct=False):
        self.ser = serial.Serial()
        self.ser.baudrate = baud
        self.ser.port = port
        self.ser.timeout = 0.5
        self.ser.open()
        time.sleep(2)
        self.error = 0
        self.hasInterpolation = interpolation
        self.direct = direct
        self.movecount = 0
        self.commandtypes = commandtypes
        self.addtypes = addtypes
        self.index = -1
        
    def flush(self):
        """ Send an instruction to a device. """
        self.ser.flushInput() 

    def sendpkt(self):
          #create packets for sending to the Arbotix_M
          for k in self.addtypes:   
             atrib[k] = atrib[k] + 128
          checksum = 0
          self.ser.write(chr(255))
          atrib['i_ComMode'] = atrib['i_Mode'] + atrib['i_Gait']
          for k in self.commandtypes: 
             self.ser.write(chr(atrib[k]))
             checksum += int(atrib[k])
          checksum = (255 - (checksum%256))
          print(atrib['i_RightV'],":_:",atrib['i_RightH'])
          self.ser.write(chr(checksum))
          move_time = time.time()
          atrib['i_leftV'] = 0
          atrib['i_leftH'] = 0
          atrib['i_RightV'] = 0
          atrib['i_RightH']= 0
        
    def getpkt(self):
           #get packets fromthe Arbotix_M       
        while (self.index < 4):
           if self.index == -1:      
             # looking for new packet
             if (self.ser.read()) == chr(255):
               self.index = 0
               checksum = 0
               print("2")
           else:
             print ("3")
             vals[self.index] = ord(self.ser.read())
             checksum += vals[self.index]
             self.index +=  1
             if(self.index == 4): # packet complete
                 if(checksum % 256 != 255):
                   print ("packet error!",(checksum % 256))
                   self.index = -1
                 else:
                   volts = vals[0]
                   temp = vals[1]
                   extra = vals[2]
        self.index = -1
        self.ser.read(999)
        print ("Output",volts,";",temp,";",extra)    
        
stdpkt  = Driver(config.stdatrib, config.addatrib)  #create standard packet                
 
def getpkt():
     stdpkt.getpkt()
     
def setgait(gait):
     if (gait != atrib['i_Gait']):
       r = 5
       while r > 0:
           stand()
           stdpkt.sendpkt()
           time.sleep(0.1)
           r -= 1 
       r = 5
       while r > 0:
           atrib['i_Gait'] = gait
           stdpkt.sendpkt()
           time.sleep(0.1)
           r -= 1
           

def stand():
     atrib['i_Mode'] = RESET;
     atrib['i_leftV'] = 0
     atrib['i_leftH'] = 0
     atrib['i_RightV'] = 0
     atrib['i_RightH']= 0
     atrib['ext']= 0
     stdpkt.sendpkt()
     
def wait(r):
     while r > 0:
      atrib['i_leftV'] = 0
      atrib['i_leftH'] = 0
      atrib['i_RightV'] = 0
      atrib['i_RightH']= 0
      atrib['ext']= 0
      time.sleep(0.1)
      r -= 1
      stdpkt.sendpkt()
      
def state(balance,doubleT,stand):
    atrib['i_Buttons'] = 0
    if(balance):
     atrib['i_Buttons'] += 1
    if(doubleT):
     atrib['i_Buttons'] += 2
    if(stand):
     atrib['i_Buttons'] += 4
    print (atrib['i_Buttons'])

def travel(angle,speed,rotate): #calculate travel related commands
     atrib['i_Mode'] = WALK
     anglerad =  math.radians(angle) 
     print("angle:>>>", angle)
     print("speed:>>>", speed)
     atrib['i_leftV'] = 0
     atrib['i_leftH'] = rotate
     atrib['i_RightV'] = int(math.cos (anglerad) * speed)
     atrib['i_RightH']= int(math.sin (anglerad) * speed)
     print("i_RightV:>>>>>>>", atrib['i_RightV'])
     atrib['ext']= 0
     stdpkt.sendpkt()

     
def rotate(left,right,up): #calculate rotate related commands
     atrib['i_Mode'] = ROTATE
     atrib['i_leftV'] = up
     atrib['i_leftH'] = left
     atrib['i_RightV'] = 0
     atrib['i_RightH']= right
     atrib['ext']= 0
     stdpkt.sendpkt()
     
def translate(left,right,up): #calculate translate related commands
     atrib['i_Mode'] = TRANSLATE 
     atrib['i_leftV'] = up
     atrib['i_leftH'] = left
     atrib['i_RightV'] = 0
     atrib['i_RightH']= right
     atrib['ext']= 0
     stdpkt.sendpkt()

                       