#!/usr/bin/env python

"""
    arbotix_low_speed_all_servos.py - Version 0.1 2013-11-29
    
    Set all servos to a low speed.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2013 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy, time
from arbotix_msgs.srv import SetSpeed

class SetSpeeds():
    def __init__(self):
        rospy.init_node('arbotix_low_speed_all_Servos')

        # The list of joints is stored in the /arbotix/joints parameter
        self.joints = rospy.get_param('/arbotix/joints', '')
        
        # Set a moderate default servo speed
        default_speed = rospy.get_param('~default_speed', 0.5)
        
        # A list to hold the set_speed services
        set_speed_services = list()
            
        for joint in self.joints:
            set_speed_service = '/' + joint + '/set_speed'
            rospy.wait_for_service(set_speed_service)  
            set_speed_services.append(rospy.ServiceProxy(set_speed_service, SetSpeed))

        # Relax all servos to give them a rest.
        for set_speed in set_speed_services:
            set_speed(default_speed)

        
if __name__=='__main__':
    try:
        SetSpeeds()
        rospy.loginfo("Servo speeds set.")
    except rospy.ROSInterruptException:
        rospy.loginfo("Oops! Exception occurred while trying to set servo speeds...")
        
