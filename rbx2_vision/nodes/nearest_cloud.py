#!/usr/bin/env python

"""
    nearest_cloud.py - Version 1.0 2013-07-28
    
    Compute the COG of the nearest object in x-y-z space and publish as a PoseStamped message.
    
    Relies on PCL ROS nodelets in the launch file to pre-filter the
    cloud on the x, y and z dimensions.
    
    Based on the follower application by Tony Pratkanis at:
    
    http://ros.org/wiki/turtlebot_follower
    
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
from roslib import message
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, PoseStamped
import numpy as np

class NearestCloud():
    def __init__(self):
        rospy.init_node("nearest_cloud")
        
        # Define the target publisher
        self.target_pub = rospy.Publisher('target_pose', PoseStamped, queue_size=5)
        
        rospy.Subscriber('point_cloud', PointCloud2, self.get_nearest_cloud)
        
        # Wait for the pointcloud topic to become available
        rospy.wait_for_message('point_cloud', PointCloud2)
        
    def get_nearest_cloud(self, msg):
        points = list()
        
        # Get all the points in the visible cloud (may be prefiltered by other nodes)
        for point in point_cloud2.read_points(msg, skip_nans=True):
            points.append(point[:3])

        # Convert to a numpy array            
        points_arr = np.float32([p for p in points]).reshape(-1, 1, 3) 
        
        # Compute the COG 
        cog = np.mean(points_arr, 0)

        # Abort if we get an NaN in any component
        if np.isnan(np.sum(cog)):
            return
        
        # Store the COG in a ROS Point object
        cog_point = Point()
        cog_point.x = cog[0][0]
        cog_point.y = cog[0][1]
        cog_point.z = cog[0][2]
        
        # Give the COG a unit orientation and store as a PoseStamped message
        target = PoseStamped()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = msg.header.frame_id
        target.pose.position = cog_point
        target.pose.orientation.w = 1.0
        
        # Publish the PoseStamped message on the /target_pose topic
        self.target_pub.publish(target)
                               
if __name__ == '__main__':
    try:
        NearestCloud()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nearest cloud node terminated.")
