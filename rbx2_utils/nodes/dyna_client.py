#!/usr/bin/env python

"""
    dyna_client.py - Version 1.0 2013-03-18
    
    Connect to a running battery_simulator node and alternate the battery level parameter
    between 100 and 0 every 10 seconds.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.

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

import rospy
import dynamic_reconfigure.client

class DynaClient():
    def __init__(self):
        rospy.init_node("dynamic_client")
        
        rospy.loginfo("Connecting to battery simulator node...")
    
        client = dynamic_reconfigure.client.Client("battery_simulator", timeout=30, config_callback=self.callback)    
    
        r = rospy.Rate(0.1)
        
        charge = True
        
        while not rospy.is_shutdown():
            if charge:
                level = 100
            else:
                level = 0
                
            charge = not charge
                
            client.update_configuration({"new_battery_level": level})
            
            r.sleep()

    def callback(self, config):
        rospy.loginfo("Battery Simulator config set to: " + str(config['new_battery_level']))

if __name__ == "__main__":
    DynaClient()
    
        