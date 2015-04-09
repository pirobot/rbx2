#!/usr/bin/env python

"""
    moveit_fk_demo.py - Version 0.1 2014-01-14
    
    Use forward kinemtatics to move the arm to a specified set of joint angles
    
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
from control_msgs.msg import GripperCommand

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the ROS node
        rospy.init_node('moveit_demo', anonymous=True)
        
        GRIPPER_OPEN = [0.05]
        GRIPPER_CLOSED = [-0.03]
        GRIPPER_NEUTRAL = [0.01]
 
        # Connect to the right_arm move group
        right_arm = moveit_commander.MoveGroupCommander('right_arm')
        
        # Connect to the right_gripper move group
        right_gripper = moveit_commander.MoveGroupCommander('right_gripper')
                
        # Get the name of the end-effector link
        end_effector_link = right_arm.get_end_effector_link()
        
        # Display the name of the end_effector link
        rospy.loginfo("The end effector link is: " + str(end_effector_link))
        
        # Set a small tolerance on joint angles
        right_arm.set_goal_joint_tolerance(0.001)
        right_gripper.set_goal_joint_tolerance(0.001)
        
        # Start the arm target in "resting" pose stored in the SRDF file
        right_arm.set_named_target('resting')
        
        # Plan a trajectory to the goal configuration
        traj = right_arm.plan()
         
        # Execute the planned trajectory
        right_arm.execute(traj)
        
        # Pause for a moment
        rospy.sleep(1)
         
        # Set the gripper target to neutal position using a joint value target
        right_gripper.set_joint_value_target(GRIPPER_NEUTRAL)
         
        # Plan and execute the gripper motion
        right_gripper.go()
        rospy.sleep(1)
         
        # Set target joint values for the arm: joints are in the order they appear in
        # the kinematic tree.
        joint_positions = [-0.0867, -1.274, 0.02832, 0.0820, -1.273, -0.003]
 
        # Set the arm's goal configuration to the be the joint positions
        right_arm.set_joint_value_target(joint_positions)
                 
        # Plan and execute the motion
        right_arm.go()
        rospy.sleep(1)
         
        # Save this configuration for later
        right_arm.remember_joint_values('saved_config', joint_positions)
         
        # Close the gripper as if picking something up
        right_gripper.set_joint_value_target(GRIPPER_CLOSED)
        right_gripper.go()
        rospy.sleep(1)
                 
        # Set the arm target to the named "straight_out" pose stored in the SRDF file
        right_arm.set_named_target('straight_forward')
         
        # Plan and execute the motion
        right_arm.go()
        rospy.sleep(1)
                  
        # Set the goal configuration to the named configuration saved earlier
        right_arm.set_named_target('saved_config')
         
        # Plan and execute the motion
        right_arm.go()
        rospy.sleep(1)
         
        # Open the gripper as if letting something go
        right_gripper.set_joint_value_target(GRIPPER_OPEN)
        right_gripper.go()
        rospy.sleep(1)
         
        # Return the arm to the named "resting" pose stored in the SRDF file
        right_arm.set_named_target('resting')
        right_arm.go()
        rospy.sleep(1)
         
        # Return the gripper target to neutral position
        right_gripper.set_joint_value_target(GRIPPER_NEUTRAL)
        right_gripper.go()
        rospy.sleep(1)
        
        # Cleanly shut down MoveIt
        moveit_commander.roscpp_shutdown()
        
        # Exit the script
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItDemo()
    except rospy.ROSInterruptException:
        pass
