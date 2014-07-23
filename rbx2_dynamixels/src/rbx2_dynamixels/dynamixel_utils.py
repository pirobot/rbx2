#!/usr/bin/env python

"""
    dynamixel_utils.py - Version 0.1 2014-01-14
    
    Utilitly functions for working with Dynamixel servos
    
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
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy, time
from std_srvs.srv import Empty
from arbotix_msgs.srv import SetSpeed

def arbotix_relax_all_servos():
    # The list of joints is stored in the /arbotix/joints parameter
    joints = rospy.get_param('/arbotix/joints', '')
        
    # Set a moderate default servo speed
    default_dynamixel_speed = 0.5
    
    # A list to hold the relax services
    relax_services = list()
    
    # A list to hold the set_speed services
    set_speed_services = list()
    
    # Connect to the relax and set_speed service for each joint
    for joint in joints:
        relax_service = '/' + joint + '/relax'
        rospy.wait_for_service(relax_service)  
        relax_services.append(rospy.ServiceProxy(relax_service, Empty))
        
        speed_service = '/' + joint + '/set_speed'
        rospy.wait_for_service(speed_service)  
        set_speed_services.append(rospy.ServiceProxy(speed_service, SetSpeed))

    # Set the servo speed to the default value
    for set_speed in set_speed_services:
        set_speed(default_dynamixel_speed)

    # Relax each servo
    for relax_servo in relax_services:
        relax_servo()

    # Do it a second time to be sure
    for relax_servo in relax_services:
        relax_servo()
       