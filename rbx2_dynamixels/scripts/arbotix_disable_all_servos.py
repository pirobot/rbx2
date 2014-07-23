#!/usr/bin/env python

"""
    arbotix_disable_all_servos.py - Version 0.1 2012-03-24
    
    Disable all servos
    
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
from arbotix_msgs.srv import Enable

class DisableServos():
    def __init__(self):
        rospy.init_node('disable_all_servos')
        
        # The list of joints is stored in the /arbotix/joints parameter
        self.joints = rospy.get_param('/arbotix/joints', '')

        # A list to hold the enable services                
        enable_services = list()
            
        for joint in self.joints:
            enable_service = '/' + joint + '/enable'
            rospy.wait_for_service(enable_service)  
            enable_services.append(rospy.ServiceProxy(enable_service, Enable))

        # Relax all servos to give them a rest.
        for enable in enable_services:
            enable(False)
        
if __name__=='__main__':
    try:
        DisableServos()
        rospy.loginfo("All servos disabled.")
    except rospy.ROSInterruptException:
        rospy.loginfo("Oops! Exception occurred while trying to disable servos...")
        
