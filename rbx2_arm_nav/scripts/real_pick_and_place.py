#!/usr/bin/env python

"""
    real_pick_and_place.py - Version 0.1 2014-01-14
    
    Use the simple_grasping perception pipeline to detect an object on a table top and 
    then control the arm to grasp the object and move it to a new location using
    the MoveIt! pick-and-place functions.
    
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
    
    The code adopted from the UBR-1 pick-and-place.py script is under the following license.

    # Copyright 2013-2014, Unbounded Robotics, Inc.
    # All rights reserved.
    #
    # Redistribution and use in source and binary forms, with or without
    # modification, are permitted provided that the following conditions
    # are met:
    #
    #  * Redistributions of source code must retain the above copyright
    #    notice, this list of conditions and the following disclaimer.
    #  * Redistributions in binary form must reproduce the above
    #    copyright notice, this list of conditions and the following
    #    disclaimer in the documentation and/or other materials provided
    #    with the distribution.
    #  * Neither the name of Unbounded Robotics, Inc. nor the names of its
    #    contributors may be used to endorse or promote products derived
    #    from this software without specific prior written permission.
    #
    # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    # "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    # LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    # FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    # COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    # INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    # BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    # LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    # CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    # LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    # ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    # POSSIBILITY OF SUCH DAMAGE.
    #
    # Author: Michael Ferguson

"""

import rospy
import sys

import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy

import argparse
import math
import actionlib
from moveit_python import PlanningSceneInterface
from grasping_msgs.msg import *

from rbx2_dynamixels.dynamixel_utils import arbotix_relax_all_servos

GROUP_NAME_ARM = 'right_arm'
GROUP_NAME_GRIPPER = 'right_gripper'

GRIPPER_FRAME = 'right_gripper_link'

GRIPPER_OPEN = [0.08]
GRIPPER_CLOSED = [-0.01]
GRIPPER_NEUTRAL = [0.01]

GRIPPER_JOINT_NAMES = ['right_gripper_finger_joint']

GRIPPER_EFFORT = [1.0]

REFERENCE_FRAME = 'base_footprint'

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        rospy.init_node('moveit_demo')
                
        # Use the planning scene object to add or remove objects
        scene = PlanningSceneInterface("base_link")
        
        # Create a scene publisher to push changes to the scene
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)
        
        # Create a publisher for displaying gripper poses
        self.gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped, queue_size=5)
        
        # Create a dictionary to hold object colors
        self.colors = dict()
                        
        # Initialize the move group for the right arm
        right_arm = MoveGroupCommander(GROUP_NAME_ARM)
        
        # Initialize the move group for the right gripper
        right_gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)
        
        # Get the name of the end-effector link
        end_effector_link = right_arm.get_end_effector_link()
 
        # Allow some leeway in position (meters) and orientation (radians)
        right_arm.set_goal_position_tolerance(0.05)
        right_arm.set_goal_orientation_tolerance(0.1)

        # Allow replanning to increase the odds of a solution
        right_arm.allow_replanning(True)
        
        # Set the right arm reference frame
        right_arm.set_pose_reference_frame(REFERENCE_FRAME)
        
        # Allow 5 seconds per planning attempt
        right_arm.set_planning_time(15)
        
        # Set a limit on the number of pick attempts before bailing
        max_pick_attempts = 5
        
        # Set a limit on the number of place attempts
        max_place_attempts = 3
                
        # Give the scene a chance to catch up
        rospy.sleep(2)
        
        # Connect to the simple_grasping find_objects action server
        rospy.loginfo("Connecting to basic_grasping_perception/find_objects...")
        find_objects = actionlib.SimpleActionClient("basic_grasping_perception/find_objects", FindGraspableObjectsAction)
        find_objects.wait_for_server()
        rospy.loginfo("...connected")
        
        # Give the scene a chance to catch up    
        rospy.sleep(1)
        
        # Start the arm in the "resting" pose stored in the SRDF file
        #right_arm.set_named_target('resting')
        #right_arm.go()
        
        # Open the gripper to the neutral position
        right_gripper.set_joint_value_target(GRIPPER_NEUTRAL)
        right_gripper.go()
       
        rospy.sleep(1)
        
        # Begin the main perception and pick-and-place loop
        while not rospy.is_shutdown():
            # Initialize the grasping goal
            goal = FindGraspableObjectsGoal()
            
            # We don't use the simple_grasping grasp planner as it does not work with our gripper
            goal.plan_grasps = False
            
            # Send the goal request to the find_objects action server which will trigger
            # the perception pipeline
            find_objects.send_goal(goal)
            
            # Wait for a result
            find_objects.wait_for_result(rospy.Duration(5.0))
            
            # The result will contain support surface(s) and objects(s) if any are detected
            find_result = find_objects.get_result()
    
            # Display the number of objects found
            rospy.loginfo("Found %d objects" % len(find_result.objects))
    
            # Remove all previous objects from the planning scene
            for name in scene.getKnownCollisionObjects():
                scene.removeCollisionObject(name, False)
            for name in scene.getKnownAttachedObjects():
                scene.removeAttachedObject(name, False)
            scene.waitForSync()
            
            # Clear the virtual object colors
            scene._colors = dict()
    
            # Use the nearest object on the table as the target
            target_pose = PoseStamped()
            target_pose.header.frame_id = REFERENCE_FRAME
            target_size = None
            the_object = None
            the_object_dist = 1.0
            count = -1
            
            # Cycle through all detected objects and keep the nearest one
            for obj in find_result.objects:
                count += 1
                scene.addSolidPrimitive("object%d"%count,
                                        obj.object.primitives[0],
                                        obj.object.primitive_poses[0],
                                        wait = False)

                # Choose the object nearest to the robot
                dx = obj.object.primitive_poses[0].position.x - args.x
                dy = obj.object.primitive_poses[0].position.y
                d = math.sqrt((dx * dx) + (dy * dy))
                if d < the_object_dist:
                    the_object_dist = d
                    the_object = count
                    
                    # Get the size of the target
                    target_size = obj.object.primitives[0].dimensions
                    
                    # Set the target pose
                    target_pose.pose = obj.object.primitive_poses[0]
                    
                    # We want the gripper to be horizontal
                    target_pose.pose.orientation.x = 0.0
                    target_pose.pose.orientation.y = 0.0
                    target_pose.pose.orientation.z = 0.0
                    target_pose.pose.orientation.w = 1.0
                
                # Make sure we found at least one object before setting the target ID
                if the_object != None:
                    target_id = "object%d"%the_object
                    
            # Insert the support surface into the planning scene
            for obj in find_result.support_surfaces:
                # Extend surface to floor
                height = obj.primitive_poses[0].position.z
                obj.primitives[0].dimensions = [obj.primitives[0].dimensions[0],
                                                2.0, # make table wider
                                                obj.primitives[0].dimensions[2] + height]
                obj.primitive_poses[0].position.z += -height/2.0
    
                # Add to scene
                scene.addSolidPrimitive(obj.name,
                                        obj.primitives[0],
                                        obj.primitive_poses[0],
                                        wait = False)
                
                # Get the table dimensions
                table_size = obj.primitives[0].dimensions
                
            # If no objects detected, try again
            if the_object == None or target_size is None:
                rospy.logerr("Nothing to grasp! try again...")
                continue
    
            # Wait for the scene to sync
            scene.waitForSync()
    
            # Set colors of the table and the object we are grabbing
            scene.setColor(target_id, 223.0/256.0, 90.0/256.0, 12.0/256.0)  # orange
            scene.setColor(find_result.objects[the_object].object.support_surface, 0.3, 0.3, 0.3, 0.7)  # grey
            scene.sendColors()
    
            # Skip pick-and-place if we are just detecting objects
            if args.objects:
                if args.once:
                    exit(0)
                else:
                    continue
    
            # Get the support surface ID
            support_surface = find_result.objects[the_object].object.support_surface
                        
            # Set the support surface name to the table object
            right_arm.set_support_surface_name(support_surface)
            
            # Specify a pose to place the target after being picked up
            place_pose = PoseStamped()
            place_pose.header.frame_id = REFERENCE_FRAME
            place_pose.pose.position.x = target_pose.pose.position.x
            place_pose.pose.position.y = 0.03
            place_pose.pose.position.z = table_size[2] + target_size[2] / 2.0 + 0.015
            place_pose.pose.orientation.w = 1.0
                            
            # Initialize the grasp pose to the target pose
            grasp_pose = target_pose
             
            # Shift the grasp pose half the size of the target to center it in the gripper
            try:
                grasp_pose.pose.position.x += target_size[0] / 2.0
                grasp_pose.pose.position.y -= 0.01
                grasp_pose.pose.position.z += target_size[2] / 2.0
            except:
                rospy.loginfo("Invalid object size so skipping")
                continue

            # Generate a list of grasps
            grasps = self.make_grasps(grasp_pose, [target_id])
     
            # Publish the grasp poses so they can be viewed in RViz
            for grasp in grasps:
                self.gripper_pose_pub.publish(grasp.grasp_pose)
                rospy.sleep(0.2)
         
            # Track success/failure and number of attempts for pick operation
            result = None
            n_attempts = 0
            
            # Set the start state to the current state
            right_arm.set_start_state_to_current_state()
             
            # Repeat until we succeed or run out of attempts
            while result != MoveItErrorCodes.SUCCESS and n_attempts < max_pick_attempts:
                result = right_arm.pick(target_id, grasps)
                n_attempts += 1
                rospy.loginfo("Pick attempt: " +  str(n_attempts))
                rospy.sleep(1.0)
             
            # If the pick was successful, attempt the place operation   
            if result == MoveItErrorCodes.SUCCESS:
                result = None
                n_attempts = 0
                  
                # Generate valid place poses
                places = self.make_places(place_pose)
                
                # Set the start state to the current state
                #right_arm.set_start_state_to_current_state()
                  
                # Repeat until we succeed or run out of attempts
                while result != MoveItErrorCodes.SUCCESS and n_attempts < max_place_attempts:
                    for place in places:
                        result = right_arm.place(target_id, place)
                        if result == MoveItErrorCodes.SUCCESS:
                            break
                    n_attempts += 1
                    rospy.loginfo("Place attempt: " +  str(n_attempts))
                    rospy.sleep(0.2)
                      
                if result != MoveItErrorCodes.SUCCESS:
                    rospy.loginfo("Place operation failed after " + str(n_attempts) + " attempts.")
            else:
                rospy.loginfo("Pick operation failed after " + str(n_attempts) + " attempts.")

            rospy.sleep(2)
            
            # Open the gripper to the neutral position
            right_gripper.set_joint_value_target(GRIPPER_NEUTRAL)
            right_gripper.go()
            
            rospy.sleep(2)
            
            # Return the arm to the "resting" pose stored in the SRDF file
            right_arm.set_named_target('resting')
            right_arm.go()
             
            rospy.sleep(2)
            
            # Give the servos a rest
            arbotix_relax_all_servos()
            
            rospy.sleep(2)
        
            if args.once:
                # Shut down MoveIt cleanly
                moveit_commander.roscpp_shutdown()
                
                # Exit the script
                moveit_commander.os._exit(0)
        
    # Get the gripper posture as a JointTrajectory
    def make_gripper_posture(self, joint_positions):
        # Initialize the joint trajectory for the gripper joints
        t = JointTrajectory()
        
        # Set the joint names to the gripper joint names
        t.joint_names = GRIPPER_JOINT_NAMES
        
        # Initialize a joint trajectory point to represent the goal
        tp = JointTrajectoryPoint()
        
        # Assign the trajectory joint positions to the input positions
        tp.positions = joint_positions
        
        # Set the gripper effort
        tp.effort = GRIPPER_EFFORT
        
        tp.time_from_start = rospy.Duration(1.0)
        
        # Append the goal point to the trajectory points
        t.points.append(tp)
        
        # Return the joint trajectory
        return t
    
    # Generate a gripper translation in the direction given by vector
    def make_gripper_translation(self, min_dist, desired, vector):
        # Initialize the gripper translation object
        g = GripperTranslation()
        
        # Set the direction vector components to the input
        g.direction.vector.x = vector[0]
        g.direction.vector.y = vector[1]
        g.direction.vector.z = vector[2]
        
        # The vector is relative to the gripper frame
        g.direction.header.frame_id = GRIPPER_FRAME
        
        # Assign the min and desired distances from the input
        g.min_distance = min_dist
        g.desired_distance = desired
        
        return g

    # Generate a list of possible grasps
    def make_grasps(self, initial_pose_stamped, allowed_touch_objects):
        # Initialize the grasp object
        g = Grasp()
        
        # Set the pre-grasp and grasp postures appropriately
        g.pre_grasp_posture = self.make_gripper_posture(GRIPPER_OPEN)
        g.grasp_posture = self.make_gripper_posture(GRIPPER_CLOSED)
                
        # Set the approach and retreat parameters as desired
        g.pre_grasp_approach = self.make_gripper_translation(0.05, 0.15, [1.0, 0.0, 0.0])
        g.post_grasp_retreat = self.make_gripper_translation(0.05, 0.15, [-1.0, 0.0, 0.0])
        
        # Set the first grasp pose to the input pose
        g.grasp_pose = initial_pose_stamped
    
        # Pitch angles to try
        pitch_vals = [0, 0.1, -0.1, 0.2, -0.2, 0.3, -0.3]
        
        # Yaw angles to try
        yaw_vals = [0]

        # A list to hold the grasps
        grasps = []

        # Generate a grasp for each pitch and yaw angle
        for y in yaw_vals:
            for p in pitch_vals:
                # Create a quaternion from the Euler angles
                q = quaternion_from_euler(0, p, y)
                
                # Set the grasp pose orientation accordingly
                g.grasp_pose.pose.orientation.x = q[0]
                g.grasp_pose.pose.orientation.y = q[1]
                g.grasp_pose.pose.orientation.z = q[2]
                g.grasp_pose.pose.orientation.w = q[3]
                
                # Set and id for this grasp (simply needs to be unique)
                g.id = str(len(grasps))
                
                # Set the allowed touch objects to the input list
                g.allowed_touch_objects = allowed_touch_objects
                
                # Don't restrict contact force
                g.max_contact_force = 0
                
                # Degrade grasp quality for increasing pitch angles
                g.grasp_quality = 1.0 - abs(p)
                
                # Append the grasp to the list
                grasps.append(deepcopy(g))
                
        # Return the list
        return grasps
    
    # Generate a list of possible place poses
    def make_places(self, init_pose):
        # Initialize the place location as a PoseStamped message
        place = PoseStamped()
        
        # Start with the input place pose
        place = init_pose
        
        # A list of x shifts (meters) to try
        x_vals = [0, 0.01, -0.01, 0.02, -0.02]
        
        # A list of y shifts (meters) to try
        y_vals = [0, 0.01, -0.01, 0.02, -0.02]
        
        pitch_vals = [0]
        
        # A list of yaw angles to try
        yaw_vals = [0]

        # A list to hold the places
        places = []
        
        # Generate a place pose for each angle and translation
        for y in yaw_vals:
            for p in pitch_vals:
                for y in y_vals:
                    for x in x_vals:
                        place.pose.position.x = init_pose.pose.position.x + x
                        place.pose.position.y = init_pose.pose.position.y + y
                        
                        # Create a quaternion from the Euler angles
                        q = quaternion_from_euler(0, p, y)
                        
                        # Set the place pose orientation accordingly
                        place.pose.orientation.x = q[0]
                        place.pose.orientation.y = q[1]
                        place.pose.orientation.z = q[2]
                        place.pose.orientation.w = q[3]
                        
                        # Append this place pose to the list
                        places.append(deepcopy(place))
        
        # Return the list
        return places
    
    # Set the color of an object
    def setColor(self, name, r, g, b, a = 0.9):
        # Initialize a MoveIt color object
        color = ObjectColor()
        
        # Set the id to the name given as an argument
        color.id = name
        
        # Set the rgb and alpha values given as input
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        
        # Update the global color dictionary
        self.colors[name] = color

    # Actually send the colors to MoveIt!
    def sendColors(self):
        # Initialize a planning scene object
        p = PlanningScene()

        # Need to publish a planning scene diff        
        p.is_diff = True
        
        # Append the colors from the global color dictionary 
        for color in self.colors.values():
            p.object_colors.append(color)
        
        # Publish the scene diff
        self.scene_pub.publish(p)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Simple demo of pick and place")
    parser.add_argument("--objects", help="Just do object perception", action="store_true")
    parser.add_argument("--all", help="Just do object perception, but insert all objects", action="store_true")
    parser.add_argument("--once", help="Run once.", action="store_true")
    parser.add_argument("--ready", help="Move the arm to the ready position.", action="store_true")
    parser.add_argument("--plan", help="Only do planning, no execution", action="store_true")
    parser.add_argument("--x", help="Recommended x offset, how far out an object should roughly be.", type=float, default=0.5)
    args, unknown = parser.parse_known_args()
    
    MoveItDemo()

    
