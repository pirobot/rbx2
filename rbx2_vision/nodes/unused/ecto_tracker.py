#!/usr/bin/env python

"""
This sample shows how to interact with ROS cloud subscribers and publishers.
"""

import sys
import ecto, ecto_ros, ecto_pcl_ros, ecto_pcl
import ecto_ros.ecto_sensor_msgs as ecto_sensor_msgs
from ecto.opts import doit as ecto_doit, scheduler_options, run_plasm
from ecto_pcl import *
from ecto_pcl_ros import *
import argparse
import time

from ecto_opencv.highgui import VideoCapture, imshow, FPSDrawer
from ecto_opencv.imgproc import Canny

class EctoPlasm():
    def __init__(self):
        ecto_ros.init(sys.argv,"ecto_tracker")
        
        parser = argparse.ArgumentParser(description='Ecto Tracker.')
        
        scheduler_options(parser)
        options = parser.parse_args()

        plasm = ecto.Plasm()
                
        voxel_grid = VoxelGrid("voxel_grid", leaf_size=0.01)
        passthru_z = PassThrough("passthru_z", filter_field_name="z", filter_limit_min=0.2, filter_limit_max=1.0)
        passthru_x = PassThrough("passthru_x", filter_field_name="x", filter_limit_min=-0.5, filter_limit_max=0.5)
        passthru_y = PassThrough("passthru_y", filter_field_name="y", filter_limit_min=-0.5, filter_limit_max=0.5)
        nan_filter = PassThrough("nan_removal")
        cropper = Cropper("cropper", x_min=-0.5, x_max=0.5, y_min=-0.5, y_max=0.5, z_min=0.0, z_max=1.0)
        #convex_hull = ConvexHull("convex_hull")
#        
        extract_stuff = ExtractPolygonalPrismData("extract_stuff", height_min=0.01, height_max=0.2)
        extract_indices = ExtractIndices("extract_indices", negative=False)
        extract_clusters = EuclideanClusterExtraction("extract_clusters", min_cluster_size=50, cluster_tolerance=0.005)
        extract_largest_cluster = ExtractLargestCluster("extract_largest_cluster")
        colorize = ColorizeClusters("colorize", max_clusters=3)
        merge = MergeClouds("merge")
        viewer = CloudViewer("viewer", window_name="Clouds!")

#        
        cloud_sub = ecto_sensor_msgs.Subscriber_PointCloud2("cloud_sub", topic_name='/camera/depth_registered/points')
        msg2cloud = ecto_pcl_ros.Message2PointCloud("msg2cloud", format=ecto_pcl.XYZRGB)
        
        cloud2msg = ecto_pcl_ros.PointCloud2Message("cloud2msg")
        cloud_pub = ecto_sensor_msgs.Publisher_PointCloud2("cloud_pub",topic_name='/ecto_pcl/sample_output')
        
#        plasm.connect(cloud_sub["output"] >> msg2cloud[:],
#                      msg2cloud[:] >> voxel_grid[:],
##                      voxel_grid[:] >> extract_indices[:],
#                      voxel_grid[:] >> cloud2msg[:],  
#                      cloud2msg[:] >> cloud_pub[:],)
#
##        plasm += [voxel_grid['output'] >> extract_clusters[:],
##                  extract_clusters[:] >> colorize["clusters"],
##                  cloud_generator[:] >> merge["input"],
##          colorize[:] >> merge["input2"],
##          colorize[:] >> viewer[:]
##          ]
#        
#

        plasm.connect(cloud_sub[:] >> msg2cloud[:],
                      msg2cloud[:] >> voxel_grid[:],
                      voxel_grid[:] >> cropper[:],
                      cropper[:] >> extract_clusters[:],
                      msg2cloud[:] >> extract_indices["input"],
                      extract_clusters[:] >> colorize["clusters"],
                      extract_indices[:] >> colorize["input"],
                      msg2cloud[:] >> merge["input"],
                      colorize[:] >> merge["input2"],
                      merge[:] >> cloud2msg[:],
                      cloud2msg[:] >> cloud_pub[:],)

#        plasm.connect(cloud_sub["output"] >> msg2cloud[:],
#                      msg2cloud[:] >> voxel_grid[:],
#                      #voxel_grid[:] >> nan_filter[:],
#                      #nan_filter[:] >> extract_indices["indices"],
#                      voxel_grid[:] >> extract_indices["indices"],
#                      msg2cloud[:] >> extract_indices["input"],)
#                      #extract_indices[:] >> extract_clusters[:],)
##                      extract_clusters[:] >> colorize["clusters"],
##                      extract_indices[:] >> colorize["input"],
##                      msg2cloud[:] >> merge["input"],
##                      colorize[:] >> merge["input2"],
##                      colorize[:] >> viewer[:],)

#        plasm.connect(cloud_sub["output"] >> msg2cloud[:],
#                      msg2cloud[:] >> voxel_grid["input"],
#                      voxel_grid["output"] >> passthru_z["input"],
#                      passthru_z["output"] >> passthru_x["input"],
#                      passthru_x["output"] >> passthru_y["input"],
#                      passthru_y["output"] >> extract_indices["input"],
#                      passthru_y["output"] >> extract_clusters["input"],
#                      extract_clusters[:] >> colorize["clusters"],
#                      extract_indices[:] >> colorize["input"],
#                      msg2cloud[:] >> merge["input"],
#                      colorize[:] >> merge["input2"],
#                      colorize[:] >> viewer[:],)
#
#                      #passthru_y["output"] >> cloud2msg[:],
#                      #cloud2msg[:] >> cloud_pub[:],)
        
        
        run_plasm(options, plasm, locals=vars())
        #sched = ecto.Scheduler(plasm)
        #sched.execute_async()

        time.sleep(10)
#        
        #ecto_doit(plasm, description="Ecto PCL Demo")
#        
#        s = ecto.Scheduler(plasm)
#        s.execute_async()
##        time.sleep(0.5)
##        s.stop()
#        s.execute_async()
##        assert s.running()
#        
#        n = 1
#        while True:
#            if n > 0:
#                self.leaf_size = 0.001
#            else:
#                self.leaf_size = 0.1
#            n *= -1
#            time.sleep(2)
        
#        for cell in plasm.cells():
#            if cell.name() == "voxel_grid":
#                cell.params.leaf_size = 0.1
                
if __name__ == "__main__":
    try:
        ecto_plasm = EctoPlasm()
    except KeyboardInterrupt:
        print "Shutting down Ecto demo."

    