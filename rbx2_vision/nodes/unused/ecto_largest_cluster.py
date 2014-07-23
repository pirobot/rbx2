#!/usr/bin/env python

"""
This sample shows how to interact with ROS cloud subscribers and publishers.
"""

import sys
import ecto, ecto_ros, ecto_pcl_ros, ecto_pcl
import ecto_ros.ecto_sensor_msgs as ecto_sensor_msgs
from ecto.opts import scheduler_options, run_plasm
from ecto_pcl import *
import argparse

class EctoPlasm():
    def __init__(self):
        ecto_ros.init(sys.argv,"ecto_pcl_demo")
        
        parser = argparse.ArgumentParser(description='Ecto Largest Cluster')
        
        scheduler_options(parser)
        options = parser.parse_args()

        plasm = ecto.Plasm()

        voxel_grid = VoxelGrid("voxel_grid", leaf_size=0.015)
        cropper = Cropper("cropper", x_min=-0.25, x_max=0.25, y_min=-0.25, y_max=0.25, z_min=0.0, z_max=1.0)
        extract_clusters = EuclideanClusterExtraction("extract_clusters", min_cluster_size=50, cluster_tolerance=0.02)
        extract_largest_cluster = ExtractLargestCluster("extract_largest_cluster")
        nan_filter = PassThrough("nan_removal")
        colorize = ColorizeClusters("colorize", max_clusters=100)
        extract_indices = ExtractIndices("extract_indices", negative=False)
      
        cloud_sub = ecto_sensor_msgs.Subscriber_PointCloud2("cloud_sub", topic_name='/camera/depth_registered/points')
        msg2cloud = ecto_pcl_ros.Message2PointCloud("msg2cloud", format=ecto_pcl.XYZRGB)
        
        cloud2msg = ecto_pcl_ros.PointCloud2Message("cloud2msg")
        cloud_pub = ecto_sensor_msgs.Publisher_PointCloud2("cloud_pub",topic_name='/ecto_pcl/cloud_filtered')
        
        output = ecto_pcl.XYZRGB
        
#        plasm.connect(cloud_sub[:] >>  msg2cloud[:],
#                      msg2cloud[:] >> nan_filter[:],
#                      nan_filter[:] >> voxel_grid[:],
#                      voxel_grid[:] >> extract_clusters[:],
#                      extract_clusters[:] >> extract_largest_cluster["clusters"],
#                      voxel_grid[:] >> extract_largest_cluster["input"],
#                      extract_largest_cluster[:] >> cloud2msg[:],
#                      cloud2msg[:] >> cloud_pub[:])
#        
#        run_plasm(options, plasm, locals=vars())

        plasm.connect(cloud_sub[:] >>  msg2cloud[:],
                      msg2cloud[:] >> nan_filter[:],
                      nan_filter[:] >> voxel_grid[:],
                      voxel_grid[:] >> extract_clusters[:],
                      extract_clusters[:] >> extract_largest_cluster["clusters"],
                      voxel_grid[:] >> extract_largest_cluster["input"],
                      extract_largest_cluster["output"] >> 'out')
        
        
        run_plasm(options, plasm, locals=output)
        
#        while True:
#            plasm.execute(niter=1)
#            print len(output)

                
if __name__ == "__main__":
    try:
        ecto_plasm = EctoPlasm()
    except KeyboardInterrupt:
        print "Shutting down Ecto demo."

    