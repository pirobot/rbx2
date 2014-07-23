#!/usr/bin/env python

""" ecto_passthrough.py - Version 0.1 2013-07-07

    Use the ecto_pcl library and three passthrough cells to crop a point cloud

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
    
    http://www.gnu.org/licenses/gpl.htmlPoint
      
"""

import sys
import ecto, ecto_ros, ecto_pcl_ros, ecto_pcl
import ecto_ros.ecto_sensor_msgs as ecto_sensor_msgs
import ecto_pcl
from ecto.opts import scheduler_options, run_plasm
import argparse

from ecto_opencv.highgui import VideoCapture, imshow, FPSDrawer
from ecto_opencv.imgproc import Canny

class EctoPlasm():
    def __init__(self):
        ecto_ros.init(sys.argv,"ecto_pcl_demo")

        # Create the argument parser
        parser = argparse.ArgumentParser(description='Ecto PCL Demo')

        # Use the parser to create scheduler options        
        scheduler_options(parser)
        options = parser.parse_args()

        # Create the overall plasm
        plasm = ecto.Plasm()
        
        # Create three PassThrough cells along the x, y and z dimensions
        passthru_z = ecto_pcl.PassThrough("passthru_z", filter_field_name="z", filter_limit_min=0.0, filter_limit_max=1.0)
        passthru_x = ecto_pcl.PassThrough("passthru_x", filter_field_name="x", filter_limit_min=-0.25, filter_limit_max=0.25)
        passthru_y = ecto_pcl.PassThrough("passthru_y", filter_field_name="y", filter_limit_min=-0.25, filter_limit_max=0.25)
      
        # Create the subscriber cell with the appropriate point cloud topic
        cloud_sub = ecto_sensor_msgs.Subscriber_PointCloud2("cloud_sub", topic_name='camera/depth_registered/points')
        
        # A cell to convert a point cloud ROS message to an Ecto/PCL point cloud object                
        msg2cloud = ecto_pcl_ros.Message2PointCloud("msg2cloud", format=ecto_pcl.XYZRGB)
        
        # A cell to convert a point cloud ROS message to an Ecto/PCL point cloud object                
        cloud2msg = ecto_pcl_ros.PointCloud2Message("cloud2msg")
        
        # A publisher cell with the desired topic name
        cloud_pub = ecto_sensor_msgs.Publisher_PointCloud2("cloud_pub",topic_name='ecto_pcl/cloud_filtered')

        # Create the plasm by connecting the cells               
        plasm.connect(cloud_sub[:] >> msg2cloud[:],
                      msg2cloud[:] >> passthru_z[:],
                      passthru_z[:] >> passthru_x[:],
                      passthru_x[:] >> passthru_y[:],
                      passthru_y[:] >> cloud2msg[:],
                      cloud2msg[:] >> cloud_pub[:])
        # Run the plasm
        run_plasm(options, plasm, locals=vars())
                
if __name__ == "__main__":
    try:
        ecto_plasm = EctoPlasm()
    except KeyboardInterrupt:
        print "Shutting down Ecto demo."

    