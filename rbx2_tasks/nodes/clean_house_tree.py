#!/usr/bin/env python

""" clean_house_tree.py - Version 1.0 2013-10-18

    Command a robot to move from "room to room" and "clean" each room appropriately.
    Uses the pi_trees package to implement a behavior tree task manager.

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
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rbx2_msgs.srv import *
from pi_trees_ros.pi_trees_ros import *
from rbx2_tasks.task_setup import *
from rbx2_tasks.clean_house_tasks_tree import *
from collections import OrderedDict
from math import pi, sqrt
import time

# A class to track global variables
class BlackBoard():
    def __init__(self):
        # A list to store rooms and tasks
        self.task_list = list()
        
        # The robot's current position on the map
        self.robot_position = Point()

# Initialize the black board
black_board = BlackBoard()

# Create a task list mapping rooms to tasks.
black_board.task_list = OrderedDict([
    ('living_room', [Vacuum(room="living_room", timer=5)]),
    ('kitchen', [Mop(room="kitchen", timer=7)]),
    ('bathroom', [Scrub(room="bathroom", timer=9), Mop(room="bathroom", timer=5)]),
    ('hallway', [Vacuum(room="hallway", timer=5)])
    ])

class UpdateTaskList(Task):
    def __init__(self, room, task, *args, **kwargs):
        name = "UPDATE_TASK_LIST_" + room.upper() + "_" + task.name.upper()
        super(UpdateTaskList, self).__init__(name)    
        
        self.name = name
        self.room = room
        self.task = task

    def run(self):
        try:
            black_board.task_list[self.room].remove(self.task)
            if len(black_board.task_list[self.room]) == 0:
                del black_board.task_list[self.room]
        except:
            pass
        
        return TaskStatus.SUCCESS
                
class CheckRoomCleaned(Task):
    def __init__(self,room):
        name = "CHECK_ROOM_CLEANED_" + room.upper()
        super(CheckRoomCleaned, self).__init__(name)    
        self.name = name
        self.room = room

    def run(self):
        try:
            if len(black_board.task_list[self.room]) != 0:
                return TaskStatus.FAILURE
        except:        
            return TaskStatus.SUCCESS

class CheckLocation(Task):
    def __init__(self, room, room_locations, *args, **kwargs):
        name = "CHECK_LOCATION_" + room.upper()
        super(CheckLocation, self).__init__(name)    
        self.name = name
        self.room = room
        self.room_locations = room_locations

    def run(self):
        wp = self.room_locations[self.room].position
        cp = black_board.robot_position
        
        distance = sqrt((wp.x - cp.x) * (wp.x - cp.x) +
                        (wp.y - cp.y) * (wp.y - cp.y) +
                        (wp.z - cp.z) * (wp.z - cp.z))
                                
        if distance < 0.15:
            status = TaskStatus.SUCCESS
        else:
            status = TaskStatus.FAILURE
            
        return status

class CleanHouse():
    def __init__(self):
        rospy.init_node('clean_house', anonymous=False)
        
        # Make sure we stop the robot when shutting down
        rospy.on_shutdown(self.shutdown)
        
        # Initialize a number of parameters and variables
        setup_task_environment(self)

        # Create a dictionary to hold navigation tasks for getting to each room
        MOVE_BASE = {}

        # Create a navigation task for each room
        for room in self.room_locations.iterkeys():
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = self.room_locations[room]
            MOVE_BASE[room] = SimpleActionTask("MOVE_BASE_" + str(room.upper()), "move_base", MoveBaseAction, goal, reset_after=True, feedback_cb=self.update_robot_position)
                        
        # Create the docking station nav task
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.docking_station_pose
        MOVE_BASE['dock'] = SimpleActionTask("MOVE_BASE_DOCK", "move_base", MoveBaseAction, goal, reset_after=True, feedback_cb=self.update_robot_position)
        
        # The root node
        BEHAVE = Sequence("BEHAVE")
        
        # The "stay healthy" selector
        STAY_HEALTHY = Selector("STAY_HEALTHY")
        
        # The "clean house" sequence
        CLEAN_HOUSE = Sequence("CLEAN_HOUSE")
        
        with STAY_HEALTHY:
            # Add the check battery condition (uses MonitorTask)
            CHECK_BATTERY = MonitorTask("CHECK_BATTERY", "battery_level", Float32, self.check_battery)
            
            # Add the recharge task (uses ServiceTask)
            CHARGE_ROBOT = ServiceTask("CHARGE_ROBOT", "battery_simulator/set_battery_level", SetBatteryLevel, 100, result_cb=self.recharge_cb)
            
            # Build the recharge sequence using inline syntax
            RECHARGE = Sequence("RECHARGE", [MOVE_BASE['dock'], CHARGE_ROBOT])
                
            # Add the check battery and recharge tasks to the stay healthy selector
            STAY_HEALTHY.add_child(CHECK_BATTERY)
            STAY_HEALTHY.add_child(RECHARGE)
        
        # Initialize a few dictionaries to hold the tasks for each room
        CLEANING_ROUTINE = {}
        CLEAN_ROOM = {}
        NAV_ROOM = {}
        CHECK_ROOM_CLEAN = {}
        CHECK_LOCATION = {}
        TASK_LIST = {}
        UPDATE_TASK_LIST = {}
                        
        # Create the clean house sequence
        for room in black_board.task_list.keys():
            # Convert the room name to upper case for consistency
            ROOM = room.upper()
        
            # Initialize the CLEANING_ROUTINE selector for this room
            CLEANING_ROUTINE[room] = Selector("CLEANING_ROUTINE_" + ROOM)
            
            # Initialize the CHECK_ROOM_CLEAN condition
            CHECK_ROOM_CLEAN[room] = CheckRoomCleaned(room)
            
            # Add the CHECK_ROOM_CLEAN condition to the CLEANING_ROUTINE selector
            CLEANING_ROUTINE[room].add_child(CHECK_ROOM_CLEAN[room])
            
            # Initialize the CLEAN_ROOM sequence for this room
            CLEAN_ROOM[room] = Sequence("CLEAN_" + ROOM)
    
            # Initialize the NAV_ROOM selector for this room                    
            NAV_ROOM[room] = Selector("NAV_ROOM_" + ROOM)
             
            # Initialize the CHECK_LOCATION condition for this room
            CHECK_LOCATION[room] = CheckLocation(room, self.room_locations)
         
            # Add the CHECK_LOCATION condition to the NAV_ROOM selector
            NAV_ROOM[room].add_child(CHECK_LOCATION[room])
             
            # Add the MOVE_BASE task for this room to the NAV_ROOM selector
            NAV_ROOM[room].add_child(MOVE_BASE[room])
             
            # Add the NAV_ROOM selector to the CLEAN_ROOM sequence
            CLEAN_ROOM[room].add_child(NAV_ROOM[room])
            
            # Initialize the TASK_LIST iterator for this room
            TASK_LIST[room] = Iterator("TASK_LIST_" + ROOM)
    
            # Add the tasks assigned to this room
            for task in black_board.task_list[room]:
                # Initialize the DO_TASK sequence for this room and task
                DO_TASK = Sequence("DO_TASK_" + ROOM + "_" + task.name)
                
                # Add a CHECK_LOCATION condition to the DO_TASK sequence
                DO_TASK.add_child(CHECK_LOCATION[room])
                
                # Add the task itself to the DO_TASK sequence
                DO_TASK.add_child(task)
                
                # Create an UPDATE_TASK_LIST task for this room and task
                UPDATE_TASK_LIST[room + "_" + task.name] = UpdateTaskList(room, task)
                
                # Add the UPDATE_TASK_LIST task to the DO_TASK sequence
                DO_TASK.add_child(UPDATE_TASK_LIST[room + "_" + task.name])
                
                # Add the DO_TASK sequence to the TASK_LIST iterator
                TASK_LIST[room].add_child(DO_TASK)
                
            # Add the room TASK_LIST iterator to the CLEAN_ROOM sequence
            CLEAN_ROOM[room].add_child(TASK_LIST[room])
                
            # Add the CLEAN_ROOM sequence to the CLEANING_ROUTINE selector
            CLEANING_ROUTINE[room].add_child(CLEAN_ROOM[room])
        
            # Add the CLEANING_ROUTINE for this room to the CLEAN_HOUSE sequence
            CLEAN_HOUSE.add_child(CLEANING_ROUTINE[room])
        
    # Build the full tree from the two subtrees
        BEHAVE.add_child(STAY_HEALTHY)
        BEHAVE.add_child(CLEAN_HOUSE)
        
        # Display the tree before execution
        print "Behavior Tree\n"
        print_tree(BEHAVE)

        rospy.loginfo("Starting simulated house cleaning test")
            
        # Run the tree
        while not rospy.is_shutdown():
            BEHAVE.run()
            rospy.sleep(0.1)
            
    def check_battery(self, msg):
        if msg.data is None:
            return TaskStatus.RUNNING
        else:
            if msg.data < self.low_battery_threshold:
                rospy.loginfo("LOW BATTERY - level: " + str(int(msg.data)))
                return TaskStatus.FAILURE
            else:
                return TaskStatus.SUCCESS
    
    def recharge_cb(self, result):
        rospy.loginfo("BATTERY CHARGED!")
        
    def update_robot_position(self, msg):
        black_board.robot_position = msg.base_position.pose.position

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")

        self.move_base.cancel_all_goals()
        
        self.cmd_vel_pub.publish(Twist())
        
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        CleanHouse()
    except rospy.ROSInterruptException:
        rospy.loginfo("House cleaning test finished.")
