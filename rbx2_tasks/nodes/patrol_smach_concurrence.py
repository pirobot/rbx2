#!/usr/bin/env python

""" patrol_smach_concurrence.py - Version 1.0 2013-04-12

    Control a robot using SMACH to patrol around a square a specified number of times
    while monitoring battery levels using the Concurrence container.

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
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import rospy
from smach import State, StateMachine, Concurrence, Container, UserData
from smach_ros import MonitorState, ServiceState, SimpleActionState, IntrospectionServer
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from rbx2_msgs.srv import *
from rbx2_tasks.task_setup import *
        
class Stop(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        pass
    
    def execute(self, userdata):
        rospy.loginfo("Shutting down the state machine")
        return 'succeeded'

class Patrol():
    def __init__(self):
        rospy.init_node('patrol_smach_concurrence', anonymous=False)
        
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # Initialize a number of parameters and variables
        setup_task_environment(self)
        
        # Track success rate of getting to the goal locations
        self.n_succeeded = 0
        self.n_aborted = 0
        self.n_preempted = 0
        
        # A variable to hold the last/current navigation goal
        self.last_nav_state = None
        
        # A flag to indicate whether or not we are rechargin
        self.recharging = False
        
        # A list to hold then navigation goa
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
            
            
        # Create a MoveBaseAction state for the docking station
        nav_goal = MoveBaseGoal()
        nav_goal.target_pose.header.frame_id = 'map'
        nav_goal.target_pose.pose = self.docking_station_pose
        nav_docking_station = SimpleActionState('move_base', MoveBaseAction, goal=nav_goal, result_cb=self.move_base_result_cb,
                                             exec_timeout=rospy.Duration(20.0),
                                             server_wait_timeout=rospy.Duration(10.0))

        # Initialize the navigation state machine
        self.sm_nav = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        
        # Add the nav states to the state machine with the appropriate transitions
        with self.sm_nav:
            StateMachine.add('NAV_STATE_0', nav_states[0], transitions={'succeeded':'NAV_STATE_1','aborted':'NAV_STATE_1'})
            StateMachine.add('NAV_STATE_1', nav_states[1], transitions={'succeeded':'NAV_STATE_2','aborted':'NAV_STATE_2'})
            StateMachine.add('NAV_STATE_2', nav_states[2], transitions={'succeeded':'NAV_STATE_3','aborted':'NAV_STATE_3'})
            StateMachine.add('NAV_STATE_3', nav_states[3], transitions={'succeeded':'NAV_STATE_4','aborted':'NAV_STATE_4'})
            StateMachine.add('NAV_STATE_4', nav_states[0], transitions={'succeeded':'','aborted':''})
        
        # Register a callback function to fire on state transitions within the sm_nav state machine
        self.sm_nav.register_transition_cb(self.nav_transition_cb, cb_args=[])

        # Initialize the recharge state machine
        self.sm_recharge = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        
        with self.sm_recharge:
            StateMachine.add('NAV_DOCKING_STATION', nav_docking_station, transitions={'succeeded':'RECHARGE_BATTERY'})
            StateMachine.add('RECHARGE_BATTERY', ServiceState('battery_simulator/set_battery_level', SetBatteryLevel, 100, response_cb=self.recharge_cb), 
                             transitions={'succeeded':''})        

        # Create the nav_patrol state machine using a Concurrence container
        self.nav_patrol = Concurrence(outcomes=['succeeded', 'recharge', 'stop'],
                                        default_outcome='succeeded',
                                        child_termination_cb=self.concurrence_child_termination_cb,
                                        outcome_cb=self.concurrence_outcome_cb)
        
        # Add the sm_nav machine and a battery MonitorState to the nav_patrol machine             
        with self.nav_patrol:
           Concurrence.add('SM_NAV', self.sm_nav)
           Concurrence.add('MONITOR_BATTERY', MonitorState("battery_level", Float32, self.battery_cb))
        
        # Create the top level state machine
        self.sm_top = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        
        # Add nav_patrol, sm_recharge and a Stop() machine to sm_top
        with self.sm_top:
            StateMachine.add('PATROL', self.nav_patrol, transitions={'succeeded':'PATROL', 'recharge':'RECHARGE', 'stop':'STOP'}) 
            StateMachine.add('RECHARGE', self.sm_recharge, transitions={'succeeded':'PATROL'})
            StateMachine.add('STOP', Stop(), transitions={'succeeded':''})

        # Create and start the SMACH introspection server
        intro_server = IntrospectionServer('patrol', self.sm_top, '/SM_ROOT')
        intro_server.start()
        
        # Execute the state machine
        sm_outcome = self.sm_top.execute()
        
        rospy.loginfo('State Machine Outcome: ' + str(sm_outcome))
                
        intro_server.stop()
    
    def nav_transition_cb(self, userdata, active_states, *cb_args):
        self.last_nav_state = active_states
        
    # Gets called when ANY child state terminates
    def concurrence_child_termination_cb(self, outcome_map):
        # If the current navigation task has succeeded, return True
        if outcome_map['SM_NAV'] == 'succeeded':
            return True
        # If the MonitorState state returns False (invalid), store the current nav goal and recharge
        if outcome_map['MONITOR_BATTERY'] == 'invalid':
            rospy.loginfo("LOW BATTERY! NEED TO RECHARGE...")
            if self.last_nav_state is not None:
                self.sm_nav.set_initial_state(self.last_nav_state, UserData())
            return True
        else:
            return False
    
    # Gets called when ALL child states are terminated
    def concurrence_outcome_cb(self, outcome_map):
        # If the battery is below threshold, return the 'recharge' outcome
        if outcome_map['MONITOR_BATTERY'] == 'invalid':
            return 'recharge'
        # Otherwise, if the last nav goal succeeded, return 'succeeded' or 'stop'
        elif outcome_map['SM_NAV'] == 'succeeded':
            self.patrol_count += 1
            rospy.loginfo("FINISHED PATROL LOOP: " + str(self.patrol_count))
            # If we have not completed all our patrols, start again at the beginning
            if self.n_patrols == -1 or self.patrol_count < self.n_patrols:
                self.sm_nav.set_initial_state(['NAV_STATE_0'], UserData())
                return 'succeeded'
            # Otherwise, we are finished patrolling so return 'stop'
            else:
                self.sm_nav.set_initial_state(['NAV_STATE_4'], UserData())
                return 'stop'
        # Recharge if all else fails
        else:
            return 'recharge'
        
    def battery_cb(self, userdata, msg):
        if msg.data < self.low_battery_threshold:
            self.recharging = True
            return False
        else:
            self.recharging = False
            return True
        
    def recharge_cb(self, userdata, response):
        return 'succeeded'
        
    def move_base_result_cb(self, userdata, status, result):
        if not self.recharging:
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

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        
        self.sm_nav.request_preempt()
        
        self.cmd_vel_pub.publish(Twist())
        
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        Patrol()
    except rospy.ROSInterruptException:
        rospy.loginfo("SMACH test finished.")
