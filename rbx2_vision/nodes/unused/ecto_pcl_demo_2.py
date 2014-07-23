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
        cropper = Cropper("cropper", x_min=-0.25, x_max=0.25, y_min=-0.25, y_max=0.25, z_min=0.0, z_max=1.0)
        
        convex_hull = ConvexHull("convex_hull")
        ror = RadiusOutlierRemoval("ror", min_neighbors=1, search_radius=1.0)
        sor = StatisticalOutlierRemoval("sor", mean_k=1)

        extract_indices = ExtractIndices("extract_indices", negative=False)
        extract_clusters = EuclideanClusterExtraction("extract_clusters", min_cluster_size=50, cluster_tolerance=0.005)
        extract_largest_cluster = ExtractLargestCluster("extract_largest_cluster")

        cloud_sub = ecto_sensor_msgs.Subscriber_PointCloud2("cloud_sub", topic_name='/camera/depth_registered/points')
        msg2cloud = ecto_pcl_ros.Message2PointCloud("msg2cloud", format=ecto_pcl.XYZRGB)
        
        cloud2msg = ecto_pcl_ros.PointCloud2Message("cloud2msg")
        cloud_pub = ecto_sensor_msgs.Publisher_PointCloud2("cloud_pub",topic_name='/ecto_pcl/sample_output')

        plasm.connect(cloud_sub[:] >> msg2cloud[:],
                      msg2cloud[:] >> extract_clusters[:],
                      extract_clusters[:] >> extract_largest_cluster["clusters"],
                      msg2cloud[:] >> extract_largest_cluster["input"],
                      extract_largest_cluster[:] >> cloud2msg[:],
                      cloud2msg[:] >> cloud_pub[:])
        
#        plasm.connect(cloud_sub[:] >> msg2cloud[:],
#                      msg2cloud[:] >> voxel_grid[:],
#                      msg2cloud[:] >> extract_indices["input"],
#                      voxel_grid[:] >> cropper[:],
#                      cropper[:] >> extract_clusters[:],
#                      extract_indices[:] >> extract_clusters[:],
#                      extract_clusters[:] >> extract_largest_cluster["clusters"],
#                      cropper[:] >>  extract_largest_cluster["input"],
#                      extract_largest_cluster[:] >> cloud2msg[:],
#                      cloud2msg[:] >> cloud_pub[:])
        
        run_plasm(options, plasm, locals=vars())
                
if __name__ == "__main__":
    try:
        ecto_plasm = EctoPlasm()
    except KeyboardInterrupt:
        print "Shutting down Ecto demo."

    