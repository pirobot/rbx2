#!/usr/bin/env python

"""
    pub_tf_frame.py - Version 1.0 2014-01-20
    
    Publish a tf frame as a PoseStamped message 
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.

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
from geometry_msgs.msg import PoseStamped

class PubFrame():
    def __init__(self):
        rospy.init_node('pub_tf_frame')
        
        # The rate at which we publish target messages
        rate = rospy.get_param('~rate', 20)
        
        # Convert the rate into a ROS rate
        r = rospy.Rate(rate)
        
        # The frame we want to track
        target_frame = rospy.get_param('~target_frame', 'right_gripper_link')

        # The target pose publisher
        target_pub = rospy.Publisher('target_pose', PoseStamped, queue_size=5)
        
        # Define the target as a PoseStamped message
        target = PoseStamped()
        
        target.header.frame_id = target_frame
        
        target.pose.position.x = 0
        target.pose.position.y = 0
        target.pose.position.z = 0
        
        target.pose.orientation.x = 0
        target.pose.orientation.y = 0
        target.pose.orientation.z = 0
        target.pose.orientation.w = 1
        
        rospy.loginfo("Publishing target on frame " + str(target_frame))
 
        while not rospy.is_shutdown():
            # Get the current timestamp
            target.header.stamp = rospy.Time.now()
            
            # Publish the target
            target_pub.publish(target)                   

            r.sleep()
                   
if __name__ == '__main__':
    try:
        target = PubFrame()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Target publisher is shut down.")
