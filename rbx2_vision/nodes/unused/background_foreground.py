#!/usr/bin/env python

""" background_foreground.py - Version 1.0 2013-10-11

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
import cv2
import cv2.cv as cv
from ros2opencv2 import ROS2OpenCV2
import numpy as np
from common import nothing, clock, draw_str

class BackgroundForeground(ROS2OpenCV2):
    def __init__(self, node_name):
        super(BackgroundForeground, self).__init__(node_name)
        
        self.node_name = node_name
        
        self.background = None
        self.foreground = None
        
        self.erode_kernel = cv2.getStructuringElement(cv2.MORPH_ERODE,(3,3))
        self.dilate_kernel = cv2.getStructuringElement(cv2.MORPH_DILATE,(3,3))
        
        self.fgbg = cv2.BackgroundSubtractorMOG(10, 5, 0.9, 0.1)
        
        self.frame_count = 0


    def process_image(self, cv_image):
        #grey = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        #grey = cv2.GaussianBlur(grey, (3, 3), 1.0)
        self.frame_count += 1
        #if self.frame_count > 100:
        mask_foreground = self.fgbg.apply(cv_image)
        mask_foreground = cv2.erode(mask_foreground, self.erode_kernel)
        #mask_rbg = cv2.cvtColor(mask_foreground, cv2.COLOR_GRAY2BGR)
        #draw = cv_image & mask_rbg
        cv2.imshow("FGBG", mask_foreground)

        return cv_image
            
        
    def diff_image(self, f0, f1, f2):
      d1 = cv2.absdiff(f2, f1)
      d2 = cv2.absdiff(f1, f0)
      #return cv2.bitwise_and(d1, d2)
      return cv2.absdiff(d1, d2)

if __name__ == '__main__':
    try:
        node_name = "background_foreground"
        BackgroundForeground(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down background foreground node."
        cv2.destroyAllWindows()