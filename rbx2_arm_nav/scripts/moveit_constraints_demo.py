#!/usr/bin/env python

"""
    moveit_constraints_demo.py - Version 0.1 2014-01-14
    
    Move the gripper while maintaining a given orientation
    
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

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander, RobotCommander
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_euler
from copy import deepcopy

GROUP_NAME_ARM = 'right_arm'
GROUP_NAME_GRIPPER = 'right_gripper'

GRIPPER_FRAME = 'right_gripper_link'

GRIPPER_OPEN = [0.03]
GRIPPER_CLOSED = [-0.03]
GRIPPER_NEUTRAL = [0.01]

GRIPPER_JOINT_NAMES = ['right_gripper_finger_joint']

GRIPPER_EFFORT = [1.0]

REFERENCE_FRAME = 'base_footprint'

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the ROS node
        rospy.init_node('moveit_demo', anonymous=True)
        
        robot = RobotCommander()

        # Connect to the right_arm move group
        right_arm = MoveGroupCommander(GROUP_NAME_ARM)
        
        # Initialize the move group for the right gripper
        right_gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)
                                
        # Increase the planning time since constraint planning can take a while
        right_arm.set_planning_time(15)
                        
        # Allow replanning to increase the odds of a solution
        right_arm.allow_replanning(True)
        
        # Set the right arm reference frame
        right_arm.set_pose_reference_frame(REFERENCE_FRAME)
                
        # Allow some leeway in position(meters) and orientation (radians)
        right_arm.set_goal_position_tolerance(0.05)
        right_arm.set_goal_orientation_tolerance(0.1)
        
        # Get the name of the end-effector link
        end_effector_link = right_arm.get_end_effector_link()
        
        # Start in the "resting" configuration stored in the SRDF file
        right_arm.set_named_target('resting')
         
        # Plan and execute a trajectory to the goal configuration
        right_arm.go()
        rospy.sleep(1)
        
        # Open the gripper
        right_gripper.set_joint_value_target(GRIPPER_NEUTRAL)
        right_gripper.go()
        rospy.sleep(1)
        
        # Set an initial target pose with the arm up and to the right
        target_pose = PoseStamped()
        target_pose.header.frame_id = REFERENCE_FRAME
        target_pose.pose.position.x = 0.237012590198
        target_pose.pose.position.y = -0.0747191267505
        target_pose.pose.position.z = 0.901578401949
        target_pose.pose.orientation.w = 1.0
         
        # Set the start state and target pose, then plan and execute
        right_arm.set_start_state(robot.get_current_state())
        right_arm.set_pose_target(target_pose, end_effector_link)
        right_arm.go()
        rospy.sleep(2)
        
        # Close the gripper
        right_gripper.set_joint_value_target(GRIPPER_CLOSED)
        right_gripper.go()
        rospy.sleep(1)
        
        # Store the current pose
        start_pose = right_arm.get_current_pose(end_effector_link)
        
        # Create a contraints list and give it a name
        constraints = Constraints()
        constraints.name = "Keep gripper horizontal"
        
        # Create an orientation constraint for the right gripper 
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = start_pose.header
        orientation_constraint.link_name = right_arm.get_end_effector_link()
        orientation_constraint.orientation.w = 1.0
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 3.14
        orientation_constraint.weight = 1.0
        
        # Append the constraint to the list of contraints
        constraints.orientation_constraints.append(orientation_constraint)
          
        # Set the path constraints on the right_arm
        right_arm.set_path_constraints(constraints)
        
        # Set a target pose for the arm        
        target_pose = PoseStamped()
        target_pose.header.frame_id = REFERENCE_FRAME
        target_pose.pose.position.x = 0.173187824708
        target_pose.pose.position.y = -0.0159929871606
        target_pose.pose.position.z = 0.692596608605
        target_pose.pose.orientation.w = 1.0

        # Set the start state and target pose, then plan and execute
        right_arm.set_start_state_to_current_state()
        right_arm.set_pose_target(target_pose, end_effector_link)
        right_arm.go()
        rospy.sleep(1)
          
        # Clear all path constraints
        right_arm.clear_path_constraints()
        
        # Open the gripper
        right_gripper.set_joint_value_target(GRIPPER_NEUTRAL)
        right_gripper.go()
        rospy.sleep(1)
        
        # Return to the "resting" configuration stored in the SRDF file
        right_arm.set_named_target('resting')

        # Plan and execute a trajectory to the goal configuration
        right_arm.go()
        rospy.sleep(1)

        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit MoveIt
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItDemo()
    except rospy.ROSInterruptException:
        pass
    
    
    
