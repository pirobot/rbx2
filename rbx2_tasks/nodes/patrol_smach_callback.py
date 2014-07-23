#!/usr/bin/env python

""" patrol_smach_callback.py - Version 1.0 2013-10-23

    Control a robot using SMACH to patrol a square area and loop a given number
    of times using a callback

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2013 Patrick Goebel.  All rights reserved.

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
from smach import State, StateMachine, UserData
from smach_ros import SimpleActionState, IntrospectionServer
from geometry_msgs.msg import Twist
from rbx2_tasks.task_setup import *
from random import Random
import os

class CheckCount(State):
    def __init__(self, n_patrols):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        
        self.n_patrols = n_patrols
        self.count = 0
    
    def execute(self, userdata):
        self.count += 1

        rospy.loginfo("Completed patrol number: " + str(self.count))

        if self.count < self.n_patrols:
            return 'succeeded'
        else:
            rospy.loginfo("All patrols completed.")
            return 'aborted'

class Patrol():
    def __init__(self):
        rospy.init_node('patrol_smach', anonymous=False)
        
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # Initialize a number of parameters and variables
        setup_task_environment(self)
        
        # Track success rate of getting to the goal locations
        self.n_succeeded = 0
        self.n_aborted = 0
        self.n_preempted = 0
        self.patrol_count = 0
        self.n_patrols = 1
        
        # A random number generator for use in the transition callback
        self.rand = Random()
        
        # A list to hold then nav_states
        nav_states = list()
        
        # Turn the waypoints into SMACH states
        for waypoint in self.waypoints:           
            nav_goal = MoveBaseGoal()
            nav_goal.target_pose.header.frame_id = 'map'
            nav_goal.target_pose.pose = waypoint
            move_base_state = SimpleActionState('move_base', MoveBaseAction, goal=nav_goal, result_cb=self.move_base_result_cb,
                                                 exec_timeout=rospy.Duration(10.0),
                                                 server_wait_timeout=rospy.Duration(10.0))
            nav_states.append(move_base_state)
        
        # Initialize the patrol state machine
        self.sm_patrol = StateMachine(outcomes=['succeeded','aborted','preempted'])
        
        # Register a callback function to fire on state transitions within the sm_patrol state machine
        self.sm_patrol.register_transition_cb(self.transition_cb, cb_args=[])
        
        # Add the states to the state machine with the appropriate transitions
        with self.sm_patrol:            
            StateMachine.add('NAV_STATE_0', nav_states[0], transitions={'succeeded':'NAV_STATE_1','aborted':'NAV_STATE_1'})
            StateMachine.add('NAV_STATE_1', nav_states[1], transitions={'succeeded':'NAV_STATE_2','aborted':'NAV_STATE_2'})
            StateMachine.add('NAV_STATE_2', nav_states[2], transitions={'succeeded':'NAV_STATE_3','aborted':'NAV_STATE_3'})
            StateMachine.add('NAV_STATE_3', nav_states[3], transitions={'succeeded':'NAV_STATE_4','aborted':'NAV_STATE_4'})
            StateMachine.add('NAV_STATE_4', nav_states[0], transitions={'succeeded':'NAV_STATE_0','aborted':'NAV_STATE_0'})
            StateMachine.add('CHECK_COUNT', CheckCount(self.n_patrols), transitions={'succeeded':'NAV_STATE_0','aborted':'','preempted':''})

        # Create and start the SMACH introspection server
        intro_server = IntrospectionServer('patrol', self.sm_patrol, '/SM_ROOT')
        intro_server.start()
        
        # Execute the state machine
        self.sm_outcome = self.sm_patrol.execute()
        
        rospy.loginfo('State Machine Outcome: ' + str(self.sm_outcome))
                
        intro_server.stop()
        
        os._exit(0)
        
    def move_base_result_cb(self, userdata, status, result):
        if status == actionlib.GoalStatus.SUCCEEDED:
            self.n_succeeded += 1
        elif status == actionlib.GoalStatus.ABORTED:
            self.n_aborted += 1
        elif status == actionlib.GoalStatus.PREEMPTED:
            self.n_preempted += 1

        try:
            rospy.loginfo("Success rate: " + str(100.0 * self.n_succeeded / (self.n_succeeded + self.n_aborted  + self.n_preempted)))
        except:
            pass
        
    def transition_cb(self, userdata, active_states, *cb_args):
        if self.rand.randint(0, 3) == 0:
            rospy.loginfo("Greetings human!")
            rospy.sleep(1)
            rospy.loginfo("Is everything OK?")
            rospy.sleep(1)
        else:
            rospy.loginfo("Nobody here.")
                    
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.sm_patrol.request_preempt()
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        Patrol()
    except rospy.ROSInterruptException:
        rospy.loginfo("SMACH test finished.")
