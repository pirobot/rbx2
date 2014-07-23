#!/usr/bin/env python

""" ecto_clusters.py - Version 0.1 2013-07-07

    Use Ecto's EuclideanClusterExtraction cell to break down a pointcloud into clusters.

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

class EctoPlasm():
    def __init__(self):
        ecto_ros.init(sys.argv,"ecto_pcl_demo")

        # Create the argument parser        
        parser = argparse.ArgumentParser(description='Ecto Euclidean Clustering')
        
        # Use the parser to create scheduler options        
        scheduler_options(parser)
        options = parser.parse_args()

        # Create the overall plasm
        plasm = ecto.Plasm()

        # A VoxelGrid cell
        voxel_grid = ecto_pcl.VoxelGrid("voxel_grid", leaf_size=0.01)
        
        # A Cropper cell
        cropper = ecto_pcl.Cropper("cropper", x_min=-0.35, x_max=0.35, y_min=-0.35, y_max=0.35, z_min=0.3, z_max=1.5)
        
        # A EuclideanClusterExtraction cell
        extract_clusters = ecto_pcl.EuclideanClusterExtraction("extract_clusters", min_cluster_size=50, cluster_tolerance=0.02)
        
        # A cell to filter NaN values from the point cloud
        nan_filter = ecto_pcl.PassThrough("nan_removal")
        
        # A cell to colorize the clusters
        colorize = ecto_pcl.ColorizeClusters("colorize", max_clusters=100)
 
        # Create the subscriber cell with the appropriate point cloud topic     
        cloud_sub = ecto_sensor_msgs.Subscriber_PointCloud2("cloud_sub", topic_name='camera/depth_registered/points')

        # A cell to convert a point cloud ROS message to an Ecto/PCL point cloud object                
        msg2cloud = ecto_pcl_ros.Message2PointCloud("msg2cloud", format=ecto_pcl.XYZRGB)

        # A cell to convert a Ecto/PCL point cloud back to a point cloud message        
        cloud2msg = ecto_pcl_ros.PointCloud2Message("cloud2msg")
        
        # A cell to convert a point cloud ROS message to an Ecto/PCL point cloud object                
        cloud_pub = ecto_sensor_msgs.Publisher_PointCloud2("cloud_pub",topic_name='ecto_pcl/cloud_filtered')
        
        # Create the plasm by connecting the cells               
        plasm.connect(cloud_sub[:] >>  msg2cloud[:],
                      msg2cloud[:] >> nan_filter[:],
                      nan_filter[:] >> voxel_grid[:],
                      voxel_grid[:] >> cropper[:],
                      cropper[:] >> extract_clusters[:],
                      extract_clusters[:] >> colorize["clusters"],
                      cropper[:] >> colorize["input"],
                      colorize[:] >> cloud2msg[:],
                      cloud2msg[:] >> cloud_pub[:])
        
        # Run the plasm
        run_plasm(options, plasm, locals=vars())
                
if __name__ == "__main__":
    try:
        ecto_plasm = EctoPlasm()
    except KeyboardInterrupt:
        print "Shutting down Ecto demo."

    