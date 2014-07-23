#!/usr/bin/env python

""" motion_history_demo.py - Version 1.0 2013-06-26

    Based on the OpenCV motempl.py sample code
    
    Extends the ros2opencv2.py script which takes care of user input and image display
    
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

MHI_DURATION = 0.1
DEFAULT_THRESHOLD = 12
MAX_TIME_DELTA = 0.05
MIN_TIME_DELTA = 0.02

class MotionHistory(ROS2OpenCV2):
    def __init__(self, node_name):
        super(MotionHistory, self).__init__(node_name)
        
        self.node_name = node_name

        self.visuals = ['input', 'frame_diff', 'motion_hist', 'motion_hist_color', 'grad_orient']
        #cv2.namedWindow('motion_templates')
        cv2.createTrackbar('visual', node_name, 2, len(self.visuals)-1, nothing)
        cv2.createTrackbar('threshold', node_name, DEFAULT_THRESHOLD, 255, nothing)
        self.motion_history = None
        
        cv.NamedWindow("Contours", cv.CV_WINDOW_NORMAL)
        cv.ResizeWindow("Contours", 640, 480)


    def process_image(self, cv_image):
        if self.motion_history == None:
            self.h, self.w = cv_image.shape[:2]
            self.prev_frame = cv_image.copy()
            self.motion_history = np.zeros((self.h, self.w), np.float32)
            self.hsv = np.zeros((self.h, self.w, 3), np.uint8)
            self.hsv[:,:,1] = 255
            self.erode_kernel = cv2.getStructuringElement(cv2.MORPH_ERODE,(3,3))
        
        color_frame = cv_image.copy()
        frame_diff = cv2.absdiff(color_frame, self.prev_frame)
        gray_diff = cv2.cvtColor(frame_diff, cv2.COLOR_BGR2GRAY)
        
        thresh = cv2.getTrackbarPos('threshold', self.node_name)
        
        ret, motion_mask = cv2.threshold(gray_diff, thresh, 1, cv2.THRESH_BINARY)
        
        motion_mask = cv2.erode(motion_mask, self.erode_kernel, iterations=2)
        motion_mask = cv2.dilate(motion_mask, self.erode_kernel, iterations=2)
                
        timestamp = clock()
        
        cv2.updateMotionHistory(motion_mask, self.motion_history, timestamp, MHI_DURATION)

        mg_mask, mg_orient = cv2.calcMotionGradient(self.motion_history, MAX_TIME_DELTA, MIN_TIME_DELTA, apertureSize=5 )
        seg_mask, seg_bounds = cv2.segmentMotion(self.motion_history, timestamp, MAX_TIME_DELTA)

        visual_name = self.visuals[cv2.getTrackbarPos('visual', self.node_name)]
        if visual_name == 'input':
            vis = cv_image.copy()
        elif visual_name == 'frame_diff':
            vis = frame_diff.copy()
        elif visual_name == 'motion_hist':
            vis = np.uint8(np.clip((self.motion_history-(timestamp-MHI_DURATION)) / MHI_DURATION, 0, 1)*255)
        elif visual_name == 'motion_hist_color':
            vis = np.uint8(np.clip((self.motion_history-(timestamp-MHI_DURATION)) / MHI_DURATION, 0, 1)*255)
            vis = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)
        elif visual_name == 'grad_orient':
            self.hsv[:,:,0] = mg_orient/2
            self.hsv[:,:,2] = mg_mask*255
            vis = cv2.cvtColor(self.hsv, cv2.COLOR_HSV2BGR)

        max_rect_area = 0
        
        for i, rect in enumerate([(0, 0, self.w, self.h)] + list(seg_bounds)):
            x, y, rw, rh = rect
            area = rw*rh
            if area < 64**2:
                continue
            if area < 640*480 and area > max_rect_area:
                max_rect_area = area
                max_rect = rect
            silh_roi   = motion_mask   [y:y+rh,x:x+rw]
            orient_roi = mg_orient     [y:y+rh,x:x+rw]
            mask_roi   = mg_mask       [y:y+rh,x:x+rw]
            mhi_roi    = self.motion_history[y:y+rh,x:x+rw]
            if cv2.norm(silh_roi, cv2.NORM_L1) < area*0.05:
                continue
            angle = cv2.calcGlobalOrientation(orient_roi, mask_roi, mhi_roi, timestamp, MHI_DURATION)
            color = ((255, 0, 0), (0, 0, 255))[i == 0]
            #draw_motion_comp(vis, rect, angle, color)

        #draw_str(vis, (20, 20), visual_name)

        display_image = cv_image.copy()


        if max_rect_area != 0:
            x, y, w, h = max_rect
            display = color_frame[y:y+h,x:x+w] 
#            #bounding_box = cv2.boundingRect(vis)
#            #print bounding_box
#        
            if visual_name == 'motion_hist':
                display = vis.copy()
            else:
                display = cv2.bitwise_and(color_frame, vis, vis)

            draw_str(vis, (20, 20), visual_name)
            
            contour_image = vis.copy()
            
            contours, hierarchy = cv2.findContours(contour_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            #ty:
            contour_points = list()
            
            if len(contours) != 0:
                for cnt in contours:
                    contour_points.append(cnt)
                
                vstack_points = np.vstack(contour_points)
                if len(vstack_points) > 5:
                    z_ellipse = cv2.fitEllipse(vstack_points)
                    cv2.ellipse(display_image, z_ellipse, (0,255,0), 2)
        
                cv2.drawContours(display_image, contours, -1, (0,255,0), 3)
            
            cv2.imshow("Contours", display_image)

        self.prev_frame = color_frame
        
        #cv2.waitKey(5)
                
        return cv_image

def draw_motion_comp(vis, (x, y, w, h), angle, color):
    cv2.rectangle(vis, (x, y), (x+w, y+h), (0, 255, 0))
    r = min(w/2, h/2)
    cx, cy = x+w/2, y+h/2
    angle = angle*np.pi/180
    cv2.circle(vis, (cx, cy), r, color, 3)
    cv2.line(vis, (cx, cy), (int(cx+np.cos(angle)*r), int(cy+np.sin(angle)*r)), color, 3)
        
def trunc(f, n):
    '''Truncates/pads a float f to n decimal places without rounding'''
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])
    
if __name__ == '__main__':
    try:
        node_name = "motion_history"
        MotionHistory(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down motion history node."
        cv2.destroyAllWindows()
