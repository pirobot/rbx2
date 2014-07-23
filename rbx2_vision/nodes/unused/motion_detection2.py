#!/usr/bin/env python

""" motion_dectection.py - Version 1.0 2013-10-12

    Uses simple frame differencing to detect motion
        
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

class MotionDetection(ROS2OpenCV2):
    def __init__(self, node_name):
        super(MotionDetection, self).__init__(node_name)
        
        self.node_name = node_name
        
        # Three three frames to compare
        self.frame_minus = None
        self.frame_now = None
        self.frame_plus = None
        
        self.erode_kernel = cv2.getStructuringElement(cv2.MORPH_ERODE,(3,3))
        self.dilate_kernel = cv2.getStructuringElement(cv2.MORPH_DILATE,(3,3))
        
        self.window_name = "Motion Detection"
        cv.NamedWindow(self.window_name, cv.CV_WINDOW_NORMAL)
        cv.ResizeWindow(self.window_name, 640, 480)


    def process_image(self, cv_image):
        
        grey = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        #grey = cv2.equalizeHist(grey)
        grey = cv2.GaussianBlur(grey, (3, 3), 1.0)
        #grey = cv2.fastNlMeansDenoising(grey,10,10,7,21)

        if self.frame_minus is None:
            self.frame_minus = grey
            
        elif self.frame_now is None:
             self.frame_now = grey
        
        elif self.frame_plus is None:
             self.frame_plus = grey
        
        else:
            #diff_image = cv2.absdiff(self.frame_plus, grey)
            diff_image = self.diff_image(self.frame_minus, self.frame_now, self.frame_plus)
            diff_image = cv2.Canny(diff_image, 15.0, 30.0)

            #ret, diff_image = cv2.threshold(diff_image, 64, 255, cv2.THRESH_BINARY)
            
            #diff_image = cv2.erode(diff_image, self.erode_kernel)
            #diff_image = cv2.dilate(diff_image, self.dilate_kernel)

            #diff_image = cv2.erode(diff_image, self.erode_kernel)
            
            #diff_image = np.abs(diff_image)**3
            
            cv2.imshow(self.window_name, diff_image)
            
            self.frame_minus = self.frame_now
            self.frame_now = self.frame_plus
            
            self.frame_plus = grey

        return cv_image
            
        
    def diff_image(self, f0, f1, f2):
      d1 = cv2.absdiff(f2, f1)
      d2 = cv2.absdiff(f1, f0)
      return cv2.bitwise_and(d1, d2)
      #return cv2.absdiff(d1, d2)

if __name__ == '__main__':
    try:
        node_name = "motion_detection"
        MotionDetection(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down motion detection node."
        cv2.destroyAllWindows()