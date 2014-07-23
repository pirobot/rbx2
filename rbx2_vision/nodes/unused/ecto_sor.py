#!/usr/bin/env python

import sys
import ecto, ecto_ros, ecto_pcl_ros, ecto_pcl
import ecto_ros.ecto_sensor_msgs as ecto_sensor_msgs
from ecto_pcl import *
from ecto.opts import scheduler_options, run_plasm
import argparse

class EctoPlasm():
    def __init__(self):
        ecto_ros.init(sys.argv,"ecto_pcl_demo")
        
        parser = argparse.ArgumentParser(description='My awesome program thing.')
        
        scheduler_options(parser)
        options = parser.parse_args()

        plasm = ecto.Plasm()

        voxel_grid = VoxelGrid("voxel_grid", leaf_size=0.01)
        cropper = Cropper("cropper", x_min=-0.25, x_max=0.25, y_min=-0.25, y_max=0.25, z_min=0.0, z_max=1.5)
        sor = StatisticalOutlierRemoval("sor", mean_k=2, stddev=1.5)

        cloud_sub = ecto_sensor_msgs.Subscriber_PointCloud2("cloud_sub", topic_name='/camera/depth_registered/points')
        msg2cloud = ecto_pcl_ros.Message2PointCloud("msg2cloud", format=ecto_pcl.XYZRGB)
        
        cloud2msg = ecto_pcl_ros.PointCloud2Message("cloud2msg")
        cloud_pub = ecto_sensor_msgs.Publisher_PointCloud2("cloud_pub",topic_name='/ecto_pcl/cloud_filtered')

        plasm.connect(cloud_sub[:] >> msg2cloud[:],
                      msg2cloud[:] >> voxel_grid[:],
                      voxel_grid[:] >> sor[:],
                      sor[:] >> cloud2msg[:],
                      cloud2msg[:] >> cloud_pub[:])
    
        run_plasm(options, plasm, locals=vars())
                
if __name__ == "__main__":
    try:
        ecto_plasm = EctoPlasm()
    except KeyboardInterrupt:
        print "Shutting down Ecto demo."

    