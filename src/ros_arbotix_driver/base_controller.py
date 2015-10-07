#!/usr/bin/env python

"""
    A base controller class for the Arduino microcontroller
    
    Borrowed heavily from Mike Feguson's ArbotiX base_controller.py code.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2010 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses
"""
import roslib; roslib.load_manifest('ros_arduino_python')
import rospy
import os

from math import sin, cos, pi
from geometry_msgs.msg import Quaternion, Twist, Pose
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
 
""" Class to receive Twist commands and publish Odometry data """
class BaseController:
    def __init__(self, ArbotixM, base_frame):
        self.arbotix = ArbotixM
        self.base_frame = base_frame
        self.rate = float(rospy.get_param("~base_controller_rate", 10))
        self.timeout = rospy.get_param("~base_controller_timeout", 1.0)
        self.stopped = False
        
        self.accel_limit = rospy.get_param('~accel_limit', 0.1)
                        
        now = rospy.Time.now()    
        self.then = now # time for determining dx/dy
        self.t_delta = rospy.Duration(1.0 / self.rate)
        self.t_next = now + self.t_delta

        # internal data        
        self.enc_left = None            # encoder readings
        self.enc_right = None
        self.x = 0                      # position in xy plane
        self.y = 0
        self.th = 0                     # rotation in radians
        self.v_left = 0
        self.v_right = 0
        self.v_des_left = 0             # cmd_vel setpoint
        self.v_des_right = 0
        self.last_cmd_vel = now

        # subscriptions
        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback)
        
        rospy.loginfo("Started base controller for a Hexapod base")
    
    def poll(self):
        now = rospy.Time.now()
        if now > self.t_next:
                            
            dt = now - self.then
            self.then = now
            dt = dt.to_sec()
            
            # Set motor speeds in encoder ticks per PID loop
            if not self.stopped:
                self.arbotix.travel(self.v_x, self.v_y, self.rotate)
                
            self.t_next = now + self.t_delta
            
    def stop(self):
        self.stopped = True
        self.arbotix.stop()
            
    def cmdVelCallback(self, req):
        # Handle velocity-based movement requests
        self.last_cmd_vel = rospy.Time.now()
        
        x = req.linear.x         # m/s
        y = req.linear.y         # m/s
        th = req.angular.z       # rad/s

            
        self.v_x = int(x *10)
        self.v_y = int(y *10)
        self.rotate = int(th *10)

        

    

    
