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
        
        self.prev_frame = None
        self.ave_diff = None
        
        self.erode_kernel = cv2.getStructuringElement(cv2.MORPH_ERODE,(3,3))
        self.dilate_kernel = cv2.getStructuringElement(cv2.MORPH_DILATE,(3,3))
        
        self.window_name = "Motion Detection"
        cv.NamedWindow(self.window_name, cv.CV_WINDOW_NORMAL)
        cv.ResizeWindow(self.window_name, 640, 480)
        
        self.kernel_size = 3
        self.scale = 1
        self.delta = 0
        self.ddepth = cv2.CV_16S

    def process_image(self, cv_image):
        
        grey = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        #grey = cv2.equalizeHist(grey)
        grey = cv2.GaussianBlur(grey, (5, 5), 1.0)
        
        #grey = cv2.Canny(grey, 15.0, 30.0)

        if self.prev_frame is None:
            self.prev_frame = grey

        diff_image = cv2.absdiff(grey, self.prev_frame)
                
#        if self.ave_diff is None:
#            self.ave_diff = np.float32(diff_image)
#        
#        cv2.accumulateWeighted(diff_image, self.ave_diff, 0.1)
#        display_ave = cv2.convertScaleAbs(self.ave_diff)
                    
        #diff_image = cv2.bitwise_and(grey, self.prev_frame)q

        #diff_image = cv2.erode(diff_image, self.erode_kernel, iterations=1)
        #diff_image = cv2.dilate(diff_image, self.dilate_kernel, iterations=5)
        #diff_image = cv2.dilate(diff_image, self.dilate_kernel, iterations=5)

        #diff_image = cv2.Laplacian(diff_image,self.ddepth,ksize = self.kernel_size,scale = self.scale,delta = self.delta)
        #diff_image = cv2.convertScaleAbs(diff_image)
        
        ret, diff_image = cv2.threshold(diff_image, 32, 255, 0)
        
        diff_image = cv2.erode(diff_image, self.erode_kernel, iterations=1)
                
        display_image= cv_image.copy()
        contour_image = diff_image.copy()
        
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

        #diff_image = cv2.Canny(diff_image, 15.0, 30.0)
        #diff_image = cv2.erode(diff_image, self.erode_kernel)

        #edges = cv2.Sobel(diff_image, cv2.CV_16S, 3, 3)

        #diff_image = cv2.dilate(diff_image, self.dilate_kernel)
        
        # Enhance the different for display purposes
        #diff_image = np.abs(diff_image)**2
        

        cv2.imshow(self.window_name, display_image)
        cv2.imshow("DIFF", diff_image)

        
        self.prev_frame = grey

        return cv_image
    
    def distance_to_cluster(self, test_point, cluster):
        min_distance = 10000
        for point in cluster:
            if point == test_point:
                continue
            # Use L1 distance since it is faster than L2
            distance = abs(test_point[0] - point[0])  + abs(test_point[1] - point[1])
            if distance < min_distance:
                min_distance = distance
        return min_distance
    
    def drop_points(self, outlier_threshold, mse_threshold):
        sum_x = 0
        sum_y = 0
        sum_z = 0
        sse = 0
        keypoints_xy = self.keypoints
        keypoints_z = self.keypoints
        n_xy = len(self.keypoints)
        n_z = n_xy
        
        if self.use_depth_for_tracking:
            if self.depth_image is None:
                return ((0, 0, 0), 0, 0, -1)
        
        # If there are no keypoints left to track, start over
        if n_xy == 0:
            return ((0, 0, 0), 0, 0, -1)
        
        # Compute the COG (center of gravity) of the cluster
        for point in self.keypoints:
            sum_x = sum_x + point[0]
            sum_y = sum_y + point[1]
        
        mean_x = sum_x / n_xy
        mean_y = sum_y / n_xy
        
        if self.use_depth_for_tracking:
            for point in self.keypoints:
                try:
                    z = cv.Get2D(self.depth_image, min(self.frame_height - 1, int(point[1])), min(self.frame_width - 1, int(point[0])))
                except:
                    continue
                z = z[0]
                # Depth values can be NaN which should be ignored
                if isnan(z):
                    continue
                else:
                    sum_z = sum_z + z
                    
            mean_z = sum_z / n_z
            
        else:
            mean_z = -1
        
        # Compute the x-y MSE (mean squared error) of the cluster in the camera plane
        for point in self.keypoints:
            sse = sse + (point[0] - mean_x) * (point[0] - mean_x) + (point[1] - mean_y) * (point[1] - mean_y)
            #sse = sse + abs((point[0] - mean_x)) + abs((point[1] - mean_y))
        
        # Get the average over the number of feature points
        mse_xy = sse / n_xy
        
        # The MSE must be > 0 for any sensible feature cluster
        if mse_xy == 0 or mse_xy > mse_threshold:
            return ((0, 0, 0), 0, 0, -1)
        
        # Throw away the outliers based on the x-y variance
        max_err = 0
        for point in self.keypoints:
            std_err = ((point[0] - mean_x) * (point[0] - mean_x) + (point[1] - mean_y) * (point[1] - mean_y)) / mse_xy
            if std_err > max_err:
                max_err = std_err
            if std_err > outlier_threshold:
                keypoints_xy.remove(point)
                if self.show_add_drop:
                    # Briefly mark the removed points in red
                    cv2.circle(self.marker_image, (point[0], point[1]), 3, (0, 0, 255), cv.CV_FILLED, 2, 0)   
                try:
                    keypoints_z.remove(point)
                    n_z = n_z - 1
                except:
                    pass
                
                n_xy = n_xy - 1
                                
        # Now do the same for depth
        if self.use_depth_for_tracking:
            sse = 0
            for point in keypoints_z:
                try:
                    z = cv.Get2D(self.depth_image, min(self.frame_height - 1, int(point[1])), min(self.frame_width - 1, int(point[0])))
                    z = z[0]
                    sse = sse + (z - mean_z) * (z - mean_z)
                except:
                    n_z = n_z - 1
            
            if n_z != 0:
                mse_z = sse / n_z
            else:
                mse_z = 0
            
            # Throw away the outliers based on depth using percent error 
            # rather than standard error since depth values can jump
            # dramatically at object boundaries
            for point in keypoints_z:
                try:
                    z = cv.Get2D(self.depth_image, min(self.frame_height - 1, int(point[1])), min(self.frame_width - 1, int(point[0])))
                    z = z[0]
                except:
                    continue
                try:
                    pct_err = abs(z - mean_z) / mean_z
                    if pct_err > self.pct_err_z:
                        keypoints_xy.remove(point)
                        if self.show_add_drop:
                            # Briefly mark the removed points in red
                            cv2.circle(self.marker_image, (point[0], point[1]), 2, (0, 0, 255), cv.CV_FILLED)  
                except:
                    pass
        else:
            mse_z = -1
        
        self.keypoints = keypoints_xy
               
        # Consider a cluster bad if we have fewer than min_keypoints left
        if len(self.keypoints) < min_keypoints:
            score = -1
        else:
            score = 1

        return ((mean_x, mean_y, mean_z), mse_xy, mse_z, score)
        

if __name__ == '__main__':
    try:
        node_name = "motion_detection"
        MotionDetection(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down motion detection node."
        cv2.destroyAllWindows()