#!/usr/bin/env python

""" monitor_dynamixels.py - Version 0.1 2013-07-07

    Monitor the /diagnostics topic for Dynamixel messages and disable a servo if we
    receive an ERROR status message (e.g. overheating).

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.htmlPoint
      
"""

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from arbotix_msgs.srv import Relax, Enable

class MonitorDynamixels:
    def __init__(self):
        # Initialize the node
        rospy.init_node("monitor_dynamixels")
        
        # The arbotix controller uses the /arbotix namespace
        namespace = '/arbotix'

        # Get the list of joints (servos)
        self.joints = rospy.get_param(namespace + '/joints', '')    
        
        # Minimum time to rest servos that are hot
        self.minimum_rest_interval = rospy.get_param('~minimum_rest_interval', 60)
        
        # Initialize the rest timer
        self.rest_timer = 0

        # Are we already resting a servo?
        self.resting = False
        
        # Are the servos enabled?
        self.servos_enabled = False
        
        # Have we displayed a warning recently?
        self.warned = False

        # Connect to the servo services
        self.connect_servos()
        
        rospy.Subscriber('diagnostics', DiagnosticArray, self.get_diagnostics)

    def get_diagnostics(self, msg):
        if self.rest_timer != 0:
            if rospy.Time.now() - self.rest_timer < rospy.Duration(self.minimum_rest_interval):
                return
            else:
                self.resting = False
                rest_timer = 0
        
        # Track if we have issued a warning on this pass
        warn = False
                
        for k in range(len(msg.status)):
            # Check for the Dynamixel identifying string in the name field
            if not '_joint' in msg.status[k].name:
                # Skip other diagnostic messages
                continue
            
            # Check the DiagnosticStatus level for this servo
            if msg.status[k].level == DiagnosticStatus.ERROR:
                # If the servo is overheating and not already resting, then disable all servos
                if not self.resting:
                    rospy.loginfo("DANGER: Overheating servo: " + str(msg.status[k].name))
                    rospy.loginfo("Disabling servos for a minimum of " + str(self.minimum_rest_interval) + " seconds...")
                    self.disable_servos()
                    self.servos_enabled = False
                    self.rest_timer = rospy.Time.now()
                    self.resting = True
                    break
            elif msg.status[k].level == DiagnosticStatus.WARN:
                # If the servo is starting to get toasty, display a warning but do not disable
                rospy.loginfo("WARNING: Servo " + str(msg.status[k].name) + " getting hot...")
                self.warned = True
                warn = True

        # No servo is overheated so re-enable all servos
        if not self.resting and not self.servos_enabled:
            rospy.loginfo("Dynamixel temperatures OK so enabling")
            self.enable_servos()
            self.servos_enabled = True
            self.resting = False
            
        # Check if a prior warning is no longer necessary
        if self.warned and not warn:
            rospy.loginfo("All servos back to a safe temperature")
            self.warned = False

        
    def connect_servos(self):
        # Create a dictionary to hold the torque and enable services
        self.relax = dict()
        self.enable = dict()

        # Connect to the set_speed services and define a position publisher for each servo
        rospy.loginfo("Waiting for joint controllers services...")
                
        for joint in sorted(self.joints):
            # A service to relax a servo
            relax = '/' + joint + '/relax'
            rospy.wait_for_service(relax) 
            self.relax[joint] = rospy.ServiceProxy(relax, Relax)
                
            # A service to enable/disable a servo          
            enable_service = '/' + joint + '/enable'
            rospy.wait_for_service(enable_service) 
            self.enable[joint] = rospy.ServiceProxy(enable_service, Enable)
                
        rospy.loginfo("Connected to servos.")
                
    def disable_servos(self):
        for joint in sorted(self.joints):
            self.relax[joint]()
            self.enable[joint](False)
                
    def enable_servos(self):
        for joint in sorted(self.joints):
            self.enable[joint](True)
            
if __name__ == '__main__':
    MonitorDynamixels()
    rospy.spin()
    