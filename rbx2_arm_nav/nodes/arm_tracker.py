#!/usr/bin/env python

"""
    arm_tracker.py - Version 0.1 2014-01-14
    
    Move the arm to point to a target on the /target_pose topic
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy

from moveit_commander import MoveGroupCommander
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose, PointStamped
from sensor_msgs.msg import JointState
import tf
from tf.transformations import quaternion_from_euler
from scipy.spatial.distance import euclidean 
import shlex, subprocess
import os
from math import sqrt, acos

GROUP_NAME_ARM = 'right_arm'
GROUP_NAME_GRIPPER = 'right_gripper'

GRIPPER_FRAME = 'right_gripper_link'

GRIPPER_OPEN = [0.03]
GRIPPER_CLOSED = [-0.02]
GRIPPER_NEUTRAL = [0.01]

GRIPPER_JOINT_NAMES = ['right_gripper_finger_joint']

GRIPPER_EFFORT = [1.0]

REFERENCE_FRAME = 'base_footprint'

class ArmTracker:
    def __init__(self):
        rospy.init_node('arm_tracker')
        
        rospy.on_shutdown(self.shutdown)
        
        # Maximum distance of the target before the arm will lower
        self.max_target_dist = 1.2
        
        # Arm length to center of gripper frame
        self.arm_length = 0.4
        
        # Distance between the last target and the new target before we move the arm
        self.last_target_threshold = 0.01
        
        # Distance between target and end-effector before we move the arm
        self.target_ee_threshold = 0.025
        
        # Initialize the move group for the right arm
        self.right_arm = MoveGroupCommander(GROUP_NAME_ARM)
        
        # Initialize the move group for the right gripper
        right_gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)
        
        # Set the reference frame for pose targets
        self.reference_frame = REFERENCE_FRAME
        
        # Keep track of the last target pose
        self.last_target_pose = PoseStamped()
        
        # Set the right arm reference frame accordingly
        self.right_arm.set_pose_reference_frame(self.reference_frame)
                        
        # Allow replanning to increase the chances of a solution
        self.right_arm.allow_replanning(False)
                
        # Set a position tolerance in meters
        self.right_arm.set_goal_position_tolerance(0.05)
        
        # Set an orientation tolerance in radians
        self.right_arm.set_goal_orientation_tolerance(0.2)
        
        # What is the end effector link?
        self.ee_link = self.right_arm.get_end_effector_link()
        
        # Create the transform listener
        self.listener = tf.TransformListener()
        
        # Queue up some tf data...
        rospy.sleep(3)
        
        # Set the gripper target to closed position using a joint value target
        right_gripper.set_joint_value_target(GRIPPER_CLOSED)
         
        # Plan and execute the gripper motion
        right_gripper.go()
        rospy.sleep(1)
                
        # Subscribe to the target topic
        rospy.wait_for_message('/target_pose', PoseStamped)
        
        # Use queue_size=1 so we don't pile up outdated target messages
        self.target_subscriber = rospy.Subscriber('/target_pose', PoseStamped, self.update_target_pose, queue_size=1)
        
        rospy.loginfo("Ready for action!")
        
        while not rospy.is_shutdown():
            try:
                target = self.target
            except:
                rospy.sleep(0.5)
                continue
                        
            # Timestamp the target with the current time
            target.header.stamp = rospy.Time()
            
            # Get the target pose in the right_arm shoulder lift frame
            #target_arm = self.listener.transformPose('right_arm_shoulder_pan_link', target)
            target_arm = self.listener.transformPose('right_arm_base_link', target)
            
            # Convert the position values to a Python list
            p0 = [target_arm.pose.position.x, target_arm.pose.position.y, target_arm.pose.position.z]
             
            # Compute the distance between the target and the shoulder link
            dist_target_shoulder = euclidean(p0, [0, 0, 0])
                                         
            # If the target is too far away, then lower the arm
            if dist_target_shoulder > self.max_target_dist:
                rospy.loginfo("Target is too far away")
                self.right_arm.set_named_target('resting')
                self.right_arm.go()
                rospy.sleep(1)
                continue
            
            # Transform the pose to the base reference frame
            target_base = self.listener.transformPose(self.reference_frame, target)
            
            # Compute the distance between the current target and the last target
            p1 = [target_base.pose.position.x, target_base.pose.position.y, target_base.pose.position.z]
            p2 = [self.last_target_pose.pose.position.x, self.last_target_pose.pose.position.y, self.last_target_pose.pose.position.z]
                    
            dist_last_target = euclidean(p1, p2)
            
            # Move the arm only if we are far enough away from the previous target
            if dist_last_target < self.last_target_threshold:
                rospy.loginfo("Still close to last target")
                rospy.sleep(0.5)
                continue
            
            # Get the pose of the end effector in the base reference frame
            ee_pose = self.right_arm.get_current_pose(self.ee_link)
            
            # Convert the position values to a Python list
            p3 = [ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z]
            
            # Compute the distance between the target and the end-effector
            dist_target = euclidean(p1, p3)
            
            # Only move the arm if we are far enough away from the target
            if dist_target < self.target_ee_threshold:
                rospy.loginfo("Already close enough to target")
                rospy.sleep(1)
                continue
            
            # We want the gripper somewhere on the line connecting the shoulder and the target.
            # Using a parametric form of the line, the parameter ranges from 0 to the
            # minimum of the arm length and the distance to the target.
            t_max = min(self.arm_length, dist_target_shoulder)
            
            # Bring it back 10% so we don't collide with the target
            t = 0.9 * t_max
            
            # Now compute the target positions from the parameter
            try:
                target_arm.pose.position.x *= (t / dist_target_shoulder)
                target_arm.pose.position.y *= (t / dist_target_shoulder)
                target_arm.pose.position.z *= (t / dist_target_shoulder)
            except:
                rospy.sleep(1)
                rospy.loginfo("Exception!")
                continue
            
            # Transform to the base_footprint frame
            target_ee = self.listener.transformPose(self.reference_frame, target_arm)
            
            # Set the target gripper orientation to be horizontal
            target_ee.pose.orientation.x = 0
            target_ee.pose.orientation.y = 0
            target_ee.pose.orientation.z = 0
            target_ee.pose.orientation.w = 1
            
            # Update the current start state
            self.right_arm.set_start_state_to_current_state()
            
            # Set the target pose for the end-effector
            self.right_arm.set_pose_target(target_ee, self.ee_link)
            
            # Plan and execute the trajectory
            success = self.right_arm.go()
            
            if success:
                # Store the current target as the last target
                self.last_target_pose = target
            
            # Pause a bit between motions to keep from locking up
            rospy.sleep(0.5)
            
                    
    def update_target_pose(self, target):
        self.target = target

    def relax_all_servos(self):
        command = 'rosrun rbx2_dynamixels arbotix_relax_all_servos.py'
        args = shlex.split(command)
        subprocess.Popen(args)
           
    def shutdown(self):
        # Stop any further target messages from being processed
        self.target_subscriber.unregister()
        
        # Stop any current arm movement
        self.right_arm.stop()
        
        # Move to the resting position
        self.right_arm.set_named_target('resting')
        self.right_arm.go()
        
        # Relax the servos
        self.relax_all_servos()
        
        os._exit(0) 

if __name__ == "__main__":
    try:
        ArmTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Arm tracker node terminated.")
    

    
    