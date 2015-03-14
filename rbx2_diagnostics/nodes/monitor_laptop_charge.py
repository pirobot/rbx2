#!/usr/bin/env python

""" monitor_laptop_charge.py - Version 0.1 2013-07-07

    A diagnostics monitor for reporting a laptop's charge level

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
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from smart_battery_msgs.msg import SmartBatteryStatus 

class MonitorLaptopCharge():
    def __init__(self):
        rospy.init_node("monitor_laptop_charge")
        
        # Get the parameters for determining WARN and ERROR status levels
        self.warn_percent = rospy.get_param("~warn_percent", 50)
        self.error_percent = rospy.get_param("~error_percent", 20)
        
        # A diagnostics publisher
        self.diag_pub = rospy.Publisher('diagnostics', DiagnosticArray, queue_size=5)
        
        # Subscribe to the laptop_charge topic
        rospy.Subscriber('laptop_charge', SmartBatteryStatus, self.pub_diagnostics)
        
        rospy.loginfo("Monitoring laptop charge...")
            
    def pub_diagnostics(self, msg):
        # Pull out the percent charge from the message
        percent_charge = msg.percentage
        
        # Initialize the diagnostics array
        diag_arr = DiagnosticArray()
        
        # Time stamp the message with the incoming stamp
        diag_arr.header.stamp = msg.header.stamp

        # Initialize the status message
        diag_msg = DiagnosticStatus()
        
        # Make the name field descriptive of what we are measuring
        diag_msg.name = "Laptop Charge"
        
        # Add a key-value pair so we can drill down to the percent charge
        diag_msg.values.append(KeyValue('percent_charge', str(percent_charge)))
        
        # Set the diagnostics level based on the current charge and the threshold
        # parameters
        if percent_charge < self.error_percent:
            diag_msg.level = DiagnosticStatus.ERROR
            diag_msg.message = 'Battery needs recharging'
        elif percent_charge < self.warn_percent:
            diag_msg.level = DiagnosticStatus.WARN
            diag_msg.message = 'Battery is below 50%'  
        else:
            diag_msg.level = DiagnosticStatus.OK
            diag_msg.message = 'Battery charge is OK'
        
        # Append the status message to the diagnostic array
        diag_arr.status.append(diag_msg)
        
        # Publish the array
        self.diag_pub.publish(diag_arr)
        
if __name__ == '__main__':
    MonitorLaptopCharge()
    rospy.spin()
    
    
        

