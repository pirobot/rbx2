#!/usr/bin/env python

"""
    name_that_face.py - Version 1.0 2013-11-10
    
    
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2013 Patrick Goebel.  All rights reserved.

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
from cob_people_detection_msgs.msg import *

class NameThatFace():
    def __init__(self):
        rospy.init_node('name_that_face')
   
        rospy.Subscriber('face_positions', DetectionArray, self.get_faces)
        
        self.last_message = None
                
    def get_faces(self, msg):
        message = ""
        
        for detection in msg.detections:
            left_right = detection.pose.pose.position.x

            message += detection.label

            if abs(left_right) < 0.1:
                message += " is straight ahead "
            elif left_right < 0:
                message += " is on my left "
            else:
                message += " is on my right "
                            
        if message == "":
            message = "Where is everyone?"
                
        if message != self.last_message:
            rospy.loginfo(message)
            self.last_message = message
            
  
if __name__ == '__main__':
    try:
        NameThatFace()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Face recognizer node terminated.")
