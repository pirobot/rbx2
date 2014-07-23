#!/usr/bin/env python

""" random_patrol_smach.py - Version 1.0 2013-04-12

    Control a robot to patrol four waypoints chosen at random

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
    
    http://www.gnu.org/licenses/gpl.htmlPoint
      
"""

import rospy
from smach import State, StateMachine
from smach_ros import SimpleActionState, IntrospectionServer
from actionlib import GoalStatus
from geometry_msgs.msg import Twist
from rbx2_tasks.task_setup import *
from random import randrange
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback

class PickWaypoint(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded'], input_keys=['waypoints'], output_keys=['waypoint_out'])
    
    def execute(self, userdata):   
        waypoint_out = randrange(len(userdata.waypoints))
        
        userdata.waypoint_out = waypoint_out
        
        rospy.loginfo("Going to waypoint " + str(waypoint_out))
    
        return 'succeeded'
        
class Nav2Waypoint(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                       input_keys=['waypoints', 'waypoint_in'])
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        # Wait up to 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))    
        
        rospy.loginfo("Connected to move_base action server")
        
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'

    def execute(self, userdata):
        self.goal.target_pose.pose = userdata.waypoints[userdata.waypoint_in]
    
        # Send the goal pose to the MoveBaseAction server
        self.move_base.send_goal(self.goal)
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # Allow 1 minute to get there
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(60)) 
        
        # If we don't get there in time, abort the goal
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
            return 'aborted'
        else:
            # We made it!
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")
            return 'succeeded'

class RandomPatrol():
    def __init__(self):
        rospy.init_node('random_patrol', anonymous=False)
        
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # Initialize a number of parameters and variables
        setup_task_environment(self)
        
        # Initialize the patrol state machine
        self.sm_patrol = StateMachine(outcomes=['succeeded','aborted','preempted'])
        
        # Set the userdata.waypoints variable to the pre-defined waypoints
        self.sm_patrol.userdata.waypoints = self.waypoints

        # Add the states to the state machine with the appropriate transitions
        with self.sm_patrol:            
            StateMachine.add('PICK_WAYPOINT', PickWaypoint(),
                             transitions={'succeeded':'NAV_WAYPOINT'},
                             remapping={'waypoint_out':'patrol_waypoint'})
            
            StateMachine.add('NAV_WAYPOINT', Nav2Waypoint(),
                             transitions={'succeeded':'PICK_WAYPOINT', 
                                          'aborted':'PICK_WAYPOINT', 
                                          'preempted':'PICK_WAYPOINT'},
                             remapping={'waypoint_in':'patrol_waypoint'})
            
        # Create and start the SMACH introspection server
        intro_server = IntrospectionServer('patrol', self.sm_patrol, '/SM_ROOT')
        intro_server.start()
        
        # Execute the state machine
        sm_outcome = self.sm_patrol.execute()
        
        rospy.loginfo('State Machine Outcome: ' + str(sm_outcome))
                
        intro_server.stop()

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        
        self.sm_patrol.request_preempt()
        
        self.cmd_vel_pub.publish(Twist())
        
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        RandomPatrol()
    except rospy.ROSInterruptException:
        rospy.loginfo("SMACH test finished.")
