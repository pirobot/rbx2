#!/usr/bin/env python

"""
This sample shows how to interact with ROS cloud subscribers and publishers.
"""

import sys
import ecto, ecto_ros, ecto_pcl_ros, ecto_pcl
import ecto_ros.ecto_sensor_msgs as ecto_sensor_msgs
from ecto_pcl import *
from ecto.opts import doit as ecto_doit, scheduler_options, run_plasm
import argparse
import time

from ecto_opencv.highgui import VideoCapture, imshow, FPSDrawer
from ecto_opencv.imgproc import Canny

class EctoPlasm():
    def __init__(self):
        ecto_ros.init(sys.argv,"ecto_pcl_demo")
        
        parser = argparse.ArgumentParser(description='My awesome program thing.')
        
        scheduler_options(parser)
        options = parser.parse_args()

        plasm = ecto.Plasm()

        voxel_grid = VoxelGrid("voxel_grid", leaf_size=0.01)
        passthru_z = PassThrough("passthru_z", filter_field_name="z", filter_limit_min=0.0, filter_limit_max=1.0)
        passthru_x = PassThrough("passthru_x", filter_field_name="x", filter_limit_min=-0.25, filter_limit_max=0.25)
        passthru_y = PassThrough("passthru_y", filter_field_name="y", filter_limit_min=-0.25, filter_limit_max=0.25)
        cropper = Cropper("cropper", x_min=-0.25, x_max=0.25, y_min=-0.25, y_max=0.25, z_min=0.0, z_max=1.0)
        viewer = CloudViewer("viewer", window_name="Clouds!")
      
        cloud_sub = ecto_sensor_msgs.Subscriber_PointCloud2("cloud_sub", topic_name='/camera/depth_registered/points')
        msg2cloud = ecto_pcl_ros.Message2PointCloud("msg2cloud", format=ecto_pcl.XYZRGB)
        
        cloud2msg = ecto_pcl_ros.PointCloud2Message("cloud2msg")
        cloud_pub = ecto_sensor_msgs.Publisher_PointCloud2("cloud_pub",topic_name='/ecto_pcl/sample_output')
               
        plasm.connect(cloud_sub[:] >> msg2cloud[:],
                      msg2cloud[:] >> voxel_grid[:],
                      voxel_grid[:] >> passthru_z[:],
                      passthru_z[:] >> passthru_x[:],
                      passthru_x[:] >> passthru_y[:],
                      passthru_y[:] >> cloud2msg[:],
                      cloud2msg[:] >> cloud_pub[:])


#        plasm.connect(cloud_sub[:] >> msg2cloud[:],
#                      msg2cloud[:] >> voxel_grid[:],
#                      voxel_grid[:] >> cropper[:],
#                      cropper[:] >> cloud2msg[:],
#                      cloud2msg[:] >> cloud_pub[:])
        
        
        run_plasm(options, plasm, locals=vars())
                
if __name__ == "__main__":
    try:
        ecto_plasm = EctoPlasm()
    except KeyboardInterrupt:
        print "Shutting down Ecto demo."

    