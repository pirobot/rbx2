#!/usr/bin/env python

"""
    moveit_speed.py - Version 0.1 2014-01-14
    
    Scale the speed of a trajectory
    
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
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import PoseStamped, Pose
from rbx2_arm_nav.arm_utils import scale_trajectory_speed

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        rospy.init_node('moveit_demo')
                
        # Initialize the move group for the right arm
        right_arm = MoveGroupCommander('right_arm')
        
        # Get the name of the end-effector link
        end_effector_link = right_arm.get_end_effector_link()
        
        # Allow replanning to increase the odds of a solution
        right_arm.allow_replanning(True)
        
        # Allow some leeway in position(meters) and orientation (radians)
        right_arm.set_goal_position_tolerance(0.02)
        right_arm.set_goal_orientation_tolerance(0.1)
        
        # Start the arm in the "resting" pose stored in the SRDF file
        right_arm.set_named_target('resting')
        right_arm.go()
        
        # Move to the named "straight_forward" pose stored in the SRDF file
        right_arm.set_named_target('straight_forward')
        right_arm.go()
        rospy.sleep(2)
        
        # Move back to the resting position at roughly 1/4 speed
        right_arm.set_named_target('resting')

        # Get back the planned trajectory
        traj = right_arm.plan()
        
        # Scale the trajectory speed by a factor of 0.25
        new_traj = scale_trajectory_speed(traj, 0.25)

        # Execute the new trajectory     
        right_arm.execute(new_traj)
        rospy.sleep(1)

        # Exit MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit the script
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItDemo()

    
    