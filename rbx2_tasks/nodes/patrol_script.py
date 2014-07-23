#!/usr/bin/env python

"""
    patrol_script.py - Version 1.0 2013-05-02
    
    Command a robot to move in a square while monitoring (simulated) battery levels
    and recharging when battery is low.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2013 Patrick Goebel.  All rights reserved.

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
from geometry_msgs.msg import Twist
from rbx2_msgs.srv import *
from std_msgs.msg import Float32
from rbx2_tasks.task_setup import *

class Patrol():
    def __init__(self):
        rospy.init_node("patrol_script")
        
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        rate = rospy.get_param("~rate", 10)

        self.tick = 1.0 / rate
        self.rate = rospy.Rate(rate)
        
        # Initialize a number of parameters and variables
        setup_task_environment(self)
        
        # Initialize the low battery flag
        self.low_battery = False
        
        # A flag to indicate that we are recharging
        self.recharging = False
        
        # Set the target location to the first waypoint
        self.location = 1
        
        # We also need to know the last location
        self.last_location = 1
        
        # Wait for the simulated battery service
        rospy.loginfo("Waiting for battery service")
        rospy.wait_for_service('/battery_simulator/set_battery_level')
        
        # Create the charge battery service proxy
        self.charge_battery = rospy.ServiceProxy('battery_simulator/set_battery_level', SetBatteryLevel)

        # Subscribe to the fake battery level topic
        rospy.wait_for_message('battery_level', Float32)
        rospy.Subscriber('battery_level', Float32, self.battery_cb)
        
        # Run the specified number of patrols
        while (self.n_patrols == -1 or self.patrol_count < self.n_patrols) and not rospy.is_shutdown():
            self.patrol()
            
    def patrol(self):
        if self.recharging:
            rospy.sleep(1)
            return
                
        # Get the waypoint pose
        waypoint = self.waypoints[self.location]
        
        # Create a MoveBaseGoal from the waypoint
        goal = MoveBaseGoal()
        goal.target_pose.pose = waypoint
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        
        self.waypoint_reached = False
        
        # Let the user know where we are going
        rospy.loginfo("Going to location: " + str(self.location))
        
        # Send the move_base goal
        self.move_base.send_goal(goal, done_cb=self.nav_done_cb, feedback_cb=self.nav_feedback_cb)
            
        rospy.loginfo("Waiting for result...")

        # This flag is set to True in the nav_done_cb() callback
        self.nav_action_finished = False
        
        # A timer to test for a timeout
        time_so_far = 0.0
        
        # We cannot use the move_base.wait_for_result() method here as it will block the entire
        # script so we break it down in time slices of duriation 1 / rate.
        while not self.nav_action_finished:
            state = self.move_base.get_state()

            if time_so_far > self.move_base_timeout:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
                return
                
            time_so_far += self.tick
            self.rate.sleep()
                
        state = self.move_base.get_state()

        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Waypoint " + str(self.location) + " reached!")

            if self.last_location == len(self.waypoints) - 1:
                rospy.loginfo("FINISHED PATROL LOOP: " + str(self.patrol_count + 1))
                self.patrol_count += 1
            
            self.last_location = self.location
            self.location = (self.location + 1) % len(self.waypoints)
                
    def nav_done_cb(self, status, result):
        if status == GoalStatus.PREEMPTED:
            rospy.loginfo("Waypoint " + str(self.location) + " PREEMPTED!")
            
        self.nav_action_finished = True

    def nav_feedback_cb(self, msg):
        if self.low_battery:
            rospy.loginfo("LOW BATTERY!")
            self.recharge()
            
    def battery_cb(self, msg):
        if msg.data < self.low_battery_threshold:
            self.low_battery = True
        else:
            self.low_battery = False
            
    def recharge(self):
        self.move_base.cancel_goal()
        self.move_base.wait_for_result(rospy.Duration(self.move_base_timeout))
            
        self.recharging = True
        
        # Create a MoveBaseGoal from the docking station pose
        goal = MoveBaseGoal()
        goal.target_pose.pose = self.docking_station_pose
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        
        self.docking_station_reached = False
                
        # Let the user know where we are going
        rospy.loginfo("Going to docking station")
                
        # Send the move_base goal
        self.move_base.send_goal(goal, feedback_cb=None)
        
        self.move_base.wait_for_result(rospy.Duration(self.move_base_timeout))
        
        state = self.move_base.get_state()
                
        rospy.loginfo("STATUS: " + str(state))
        if state == GoalStatus.SUCCEEDED:
            self.docking_station_reached = True
            #self.move_base.cancel_goal()

        if self.docking_station_reached:
            rospy.loginfo("CHARGING BATTERY...")
            
            # Simulate charging the battery to full strength
            self.charge_battery(100)
            
            # Simulate how long it takes to charge
            rospy.sleep(1.0)
            
            rospy.loginfo("BATTERY CHARGED!")
            
            self.recharging = False
        
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        
        # Cancel all move_base goals
        self.move_base.cancel_all_goals()
    
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        
        rospy.sleep(1)
        
if __name__ == '__main__':
    Patrol()
    