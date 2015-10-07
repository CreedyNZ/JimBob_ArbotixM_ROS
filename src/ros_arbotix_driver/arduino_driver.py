#!/usr/bin/env python

"""
    A Python driver for the Arduino microcontroller running the
    ROSArduinoBridge firmware.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html

"""

import thread
from math import pi as PI, degrees, radians
import os
import time
import sys, traceback
import config
from serial.serialutil import SerialException
from serial import Serial

#Values for Phoenix Code Mcde setting
RESET  = 0
WALK = 10
ROTATE = 20
TRANSLATE = 30
SINLEG = 40

atrib = config.atrib

class ArbotixM:
    
    def __init__(self, port, baudrate, timeout):

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.encoder_count = 0
        self.writeTimeout = timeout
        self.interCharTimeout = timeout / 30.

        #for the controller
        self.commandtypes = config.stdatrib
        self.addtypes = config.addatrib
        self.index = -1
    
        # Keep things thread safe
        self.mutex = thread.allocate_lock()



    
    def connect(self):
        try:
            print "Connecting to Arduino on port", self.port, "..."
            self.port = Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout, writeTimeout=self.writeTimeout)
            # The next line is necessary to give the firmware time to wake up.
            time.sleep(1)
            print "Arbotix is ready."

        except SerialException:
            print "Serial Exception:"
            print sys.exc_info()
            print "Traceback follows:"
            traceback.print_exc(file=sys.stdout)
            print "Cannot connect to Arduino!"
            os._exit(1)

    def open(self): 
        ''' Open the serial port.
        '''
        self.port.open()

    def close(self): 
        ''' Close the serial port.
        '''
        self.port.close() 
    
    def send(self, cmds):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        self.port.write(cmds)

    def recv(self, timeout=0.5):
        timeout = min(timeout, self.timeout)
        ''' This command should not be used on its own: it is called by the execute commands   
            below in a thread safe manner.  Note: we use read() instead of readline() since
            readline() tends to return garbage characters from the Arduino
        '''
        c = ''
        value = ''
        attempts = 0
        while c != '\r':
            c = self.port.read(1)
            value += c
            attempts += 1
            if attempts * self.interCharTimeout > timeout:
                return None

        value = value.strip('\r')

        return value
            
    def recv_ack(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        ack = self.recv(self.timeout)
        return ack == 'OK'

    def recv_int(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        value = self.recv(self.timeout)
        try:
            return int(value)
        except:
            return None

    def execute_commander(self, cmds):
        ''' Thread safe execution of "cmd" on the Arduino returning a single integer value.
        '''
        self.mutex.acquire()

        try:
            self.port.flushInput()
        except:
            pass

        try:
            #create packets for sending to the Arbotix_M
          for k in self.addtypes:
             cmds[k] = cmds[k] + 128
          checksum = 0
          #self.port.write(chr(255))   <<<<<<<<<<<<<<<Debug
          cmds['i_ComMode'] = cmds['i_Mode'] + cmds['i_Gait']
          for k in self.commandtypes:
             #self.port.write(chr(cmds[k]))  <<<<<<<<<<<<<<<Debug
             checksum += int(cmds[k])
          checksum = (255 - (checksum%256))
          print(cmds['i_RightV'],":_:",cmds['i_RightH'])
          #self.port.write(chr(checksum))  <<<<<<<<<<<<<<<Debug
          cmds['i_leftV'] = 0
          cmds['i_leftH'] = 0
          cmds['i_RightV'] = 0
          cmds['i_RightH']= 0
        except:
            self.mutex.release()
            print "Exception executing command: " + cmds
            value = None

        self.mutex.release()
        return int()

    def execute_array(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning an array.
        '''
        self.mutex.acquire()
        
        try:
            self.port.flushInput()
        except:
            pass
        
        ntries = 1
        attempts = 0
        
        try:
            self.port.write(cmd + '\r')
            values = self.recv_array()
            while attempts < ntries and (values == '' or values == 'Invalid Command' or values == [] or values == None):
                try:
                    self.port.flushInput()
                    self.port.write(cmd + '\r')
                    values = self.recv_array()
                except:
                    print("Exception executing command: " + cmd)
                attempts += 1
        except:
            self.mutex.release()
            print "Exception executing command: " + cmd
            raise SerialException
            return []
        
        try:
            values = map(int, values)
        except:
            values = []

        self.mutex.release()
        return values
        
    def execute_ack(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning True if response is ACK.
        '''
        self.mutex.acquire()
        
        try:
            self.port.flushInput()
        except:
            pass
        
        ntries = 1
        attempts = 0
        
        try:
            self.port.write(cmd + '\r')
            ack = self.recv(self.timeout)
            while attempts < ntries and (ack == '' or ack == 'Invalid Command' or ack == None):
                try:
                    self.port.flushInput()
                    self.port.write(cmd + '\r')
                    ack = self.recv(self.timeout)
                except:
                    print "Exception executing command: " + cmd
            attempts += 1
        except:
            self.mutex.release()
            print "execute_ack exception when executing", cmd
            print sys.exc_info()
            return 0
        
        self.mutex.release()
        return ack == 'OK'   


    def travel(self,x,y,z): #calculate travel related commands
        atrib['i_Mode'] = WALK
        atrib['i_leftV'] = 0
        atrib['i_leftH'] = z
        atrib['i_RightV'] = x
        atrib['i_RightH']= y
        atrib['ext']= 0
        self.execute_commander(atrib)
        
    def stop(self):
        ''' Stop both motors.
        '''
        atrib['i_Mode'] = WALK
        atrib['i_leftV'] = 0
        atrib['i_leftH'] = 0
        atrib['i_RightV'] = 0
        atrib['i_RightH']= 0
        atrib['ext']= 0
        self.execute_commander(atrib)


    
