#!/usr/bin/env python

"""
    people_monitor.py - Version 1.0 2013-11-16
    

    
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
import thread
from cob_people_detection_msgs.msg import *
from scipy.spatial.distance import euclidean
from math import sqrt

class PeopleMonitor():
    def __init__(self):
        rospy.init_node("people_monitor")
        
        rate = rospy.Rate(rospy.get_param('~rate', 1.0))
                
        self.mutex = thread.allocate_lock()
                
        rospy.Subscriber('people', DetectionArray, self.track_faces)
        
        rospy.Subscriber('recognitions', DetectionArray, self.head_or_face)
        
        self.people = {}
        self.unknown = list()
        
        while not rospy.is_shutdown():
            message = "Known people: "
            message += str(self.people.keys())
            message += " N Unknown: " + str(len(self.unknown))
            
            rospy.loginfo(message)
            
            rate.sleep()
        
    def head_or_face(self, msg):
        self.unknown = list()
        
        for detection in msg.detections:
            pose = detection.pose.pose

            label = detection.label
            detector = detection.detector
            
            for i in range(len(self.unknown)):
                if self.already_tracking(self.unknown[i]):
                    del self.unknown[i]  
            
            if label == "UnknownHead":
                if not self.already_tracking(pose):
                    self.unknown.append(pose)           
                
    def track_faces(self, msg):
        self.people = {}

        for detection in msg.detections:
            pose = detection.pose.pose

            label = detection.label
            detector = detection.detector
            
            self.people[label] = pose
            
    def already_tracking(self, new_pose):
        p1 = [new_pose.position.x, new_pose.position.y, new_pose.position.z]

        for person, pose in self.people.iteritems():
            p2 = [pose.position.x, pose.position.y, pose.position.z]
            distance = euclidean(p1, p2)
            if distance < 0.05:
                return True
            
        for pose in self.unknown:
            p2 = [pose.position.x, pose.position.y, pose.position.z]
            distance = euclidean(p1, p2)
            if distance < 0.05:
                return True 
            
        return False
  
if __name__ == '__main__':
    try:
        PeopleMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Face recognizer node terminated.")
        
