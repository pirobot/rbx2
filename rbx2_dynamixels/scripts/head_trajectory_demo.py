#!/usr/bin/env python

"""
    head_trajectory_demo.py - Version 0.1 2014-01-14
    
    Send a trajectory to the FollowJointTrajectoryAction server
    
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
import actionlib

# Messages and actions needed to control the arm
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class HeadTrajectoryDemo():
    def __init__(self):
        rospy.init_node('head_trajectory_demo')

        # Set to True to move back to the starting configurations    
        reset = rospy.get_param('~reset', False)
        
        # Set the duration of the trajectory in seconds
        duration = rospy.get_param('~duration', 3.0)
                
        # Which joints define the head?
        head_joints = ['head_pan_joint', 'head_tilt_joint']
        
        # Set a goal configuration for the head
        if reset:
            head_goal = [0, 0]
        else:
            head_goal = [-1.3, -0.3]
        
        # Connect to the head trajectory action server
        rospy.loginfo('Waiting for head trajectory controller...')
    
        head_client = actionlib.SimpleActionClient('head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
       
        head_client.wait_for_server()
        
        rospy.loginfo('...connected.')    
        
        # Create a head trajectory with a single end point using the head_goal positions
        trajectory = JointTrajectory()
        trajectory.joint_names = head_joints
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = head_goal
        trajectory.points[0].velocities = [0.0 for i in head_joints]
        trajectory.points[0].accelerations = [0.0 for i in head_joints]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
            
        # Send the trajectory to the head action server
        rospy.loginfo('Moving the head to goal position...')
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = rospy.Duration(0.0)
    
        # Send the goal
        head_client.send_goal(goal)
        
        # Wait for up to 5 seconds for the motion to complete 
        head_client.wait_for_result(rospy.Duration(5.0))
        
        rospy.loginfo('...done')
        
if __name__ == '__main__':
    try:
        HeadTrajectoryDemo()
    except rospy.ROSInterruptException:
        pass