#!/usr/bin/env python

""" clean_house_smach.py - Version 1.0 2013-04-20

    Control a robot to move from "room to room" and "clean" each room appropriately.

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
import smach
from smach import State, StateMachine
from smach_ros import SimpleActionState, IntrospectionServer
from geometry_msgs.msg import Twist
from rbx2_tasks.task_setup import *
import easygui
import datetime
from collections import OrderedDict

# A list of rooms and tasks
task_list = {'living_room':['vacuum_floor'], 'kitchen':['mop_floor'], 'bathroom':['scrub_tub', 'mop_floor'], 'hallway':['vacuum_floor']}

class VacuumFloor(State):
    def __init__(self, room, timer):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        
        self.task = 'vacuum_floor'
        self.room = room
        self.timer = timer
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

    def execute(self, userdata):
        rospy.loginfo('Vacuuming the floor in the ' + str(self.room))
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.05
        counter = self.timer
        while counter > 0:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            self.cmd_vel_pub.publish(cmd_vel_msg)
            cmd_vel_msg.linear.x *= -1
            rospy.loginfo(counter)
            counter -= 1
            rospy.sleep(1)
        
        self.cmd_vel_pub.publish(Twist())
        message = "Finished vacuuming the " + str(self.room) + "!"
        rospy.loginfo(message)
        easygui.msgbox(message, title="Succeeded")
        
        update_task_list(self.room, self.task)

        return 'succeeded'

class MopFloor(State):
    def __init__(self, room, timer):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        
        self.task = 'mop_floor'
        self.room = room
        self.timer = timer
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
   
    def execute(self, userdata):
        rospy.loginfo('Mopping the floor in the ' + str(self.room))
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.05
        cmd_vel_msg.angular.z = 1.2
        counter = self.timer
        while counter > 0:
            self.cmd_vel_pub.publish(cmd_vel_msg)
            cmd_vel_msg.linear.x *= -1
            rospy.loginfo(counter)
            counter -= 1
            rospy.sleep(1)
            
        self.cmd_vel_pub.publish(Twist())
        message = "Done mopping the " + str(self.room) + "!"
        rospy.loginfo(message)
        easygui.msgbox(message, title="Succeeded")
        
        update_task_list(self.room, self.task)
             
        return 'succeeded'

class ScrubTub(State):
    def __init__(self, room, timer):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        
        self.task = 'scrub_tub'
        self.room = room
        self.timer = timer
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

    def execute(self, userdata):
        rospy.loginfo('Cleaning the tub...')
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.4
        cmd_vel_msg.angular.z = 0.2
        counter = self.timer
        while counter > 0:
            self.cmd_vel_pub.publish(cmd_vel_msg)
            cmd_vel_msg.linear.x *= -1
            if counter % 2 == 5:
                cmd_vel_msg.angular.z *= -1
            rospy.loginfo(counter)
            counter -= 1
            rospy.sleep(0.2)
    
        self.cmd_vel_pub.publish(Twist())
        message = "The tub is clean!"
        rospy.loginfo(message)
        easygui.msgbox(message, title="Succeeded")
        
        update_task_list(self.room, self.task)
            
        return 'succeeded'

def update_task_list(room, task):
    task_list[room].remove(task)
    if len(task_list[room]) == 0:
        del task_list[room]

class main():
    def __init__(self):
        rospy.init_node('clean_house', anonymous=False)
        
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # Initialize a number of parameters and variables
        setup_task_environment(self)
        
        # Turn the room locations into SMACH move_base action states
        nav_states = {}
        
        for room in self.room_locations.iterkeys():         
            nav_goal = MoveBaseGoal()
            nav_goal.target_pose.header.frame_id = 'map'
            nav_goal.target_pose.pose = self.room_locations[room]
            move_base_state = SimpleActionState('move_base', MoveBaseAction, goal=nav_goal, result_cb=self.move_base_result_cb, 
                                                exec_timeout=rospy.Duration(15.0),
                                                server_wait_timeout=rospy.Duration(10.0))
            nav_states[room] = move_base_state

        ''' Create individual state machines for assigning tasks to each room '''

        # Create a state machine for the living room subtask(s)
        sm_living_room = StateMachine(outcomes=['succeeded','aborted','preempted'])
        
        # Then add the subtask(s)
        with sm_living_room:
            StateMachine.add('VACUUM_FLOOR', VacuumFloor('living_room', 5), transitions={'succeeded':'','aborted':'','preempted':''})

        # Create a state machine for the kitchen subtask(s)
        sm_kitchen = StateMachine(outcomes=['succeeded','aborted','preempted'])
        
        # Then add the subtask(s)
        with sm_kitchen:
            StateMachine.add('MOP_FLOOR', MopFloor('kitchen', 5), transitions={'succeeded':'','aborted':'','preempted':''})

        # Create a state machine for the bathroom subtask(s)
        sm_bathroom = StateMachine(outcomes=['succeeded','aborted','preempted'])
        
        # Then add the subtasks
        with sm_bathroom:
            StateMachine.add('SCRUB_TUB', ScrubTub('bathroom', 7), transitions={'succeeded':'MOP_FLOOR'})
            StateMachine.add('MOP_FLOOR', MopFloor('bathroom', 5), transitions={'succeeded':'','aborted':'','preempted':''})

        # Create a state machine for the hallway subtask(s)
        sm_hallway = StateMachine(outcomes=['succeeded','aborted','preempted'])
        
        # Then add the subtasks
        with sm_hallway:
            StateMachine.add('VACUUM_FLOOR', VacuumFloor('hallway', 5), transitions={'succeeded':'','aborted':'','preempted':''})

        # Initialize the overall state machine
        sm_clean_house = StateMachine(outcomes=['succeeded','aborted','preempted'])
            
        # Build the clean house state machine from the nav states and room cleaning states
        with sm_clean_house:            
            StateMachine.add('START', nav_states['hallway'], transitions={'succeeded':'LIVING_ROOM','aborted':'LIVING_ROOM','preempted':'LIVING_ROOM'})
            
            ''' Add the living room subtask(s) '''
            StateMachine.add('LIVING_ROOM', nav_states['living_room'], transitions={'succeeded':'LIVING_ROOM_TASKS','aborted':'KITCHEN','preempted':'KITCHEN'})
            
            # When the tasks are done, continue on to the kitchen
            StateMachine.add('LIVING_ROOM_TASKS', sm_living_room, transitions={'succeeded':'KITCHEN','aborted':'KITCHEN','preempted':'KITCHEN'})
            
            ''' Add the kitchen subtask(s) '''
            StateMachine.add('KITCHEN', nav_states['kitchen'], transitions={'succeeded':'KITCHEN_TASKS','aborted':'BATHROOM','preempted':'BATHROOM'})
            
            # When the tasks are done, continue on to the bathroom
            StateMachine.add('KITCHEN_TASKS', sm_kitchen, transitions={'succeeded':'BATHROOM','aborted':'BATHROOM','preempted':'BATHROOM'})
            
            ''' Add the bathroom subtask(s) '''
            StateMachine.add('BATHROOM', nav_states['bathroom'], transitions={'succeeded':'BATHROOM_TASKS','aborted':'HALLWAY','preempted':'HALLWAY'})
            
            # When the tasks are done, return to the hallway
            StateMachine.add('BATHROOM_TASKS', sm_bathroom, transitions={'succeeded':'HALLWAY','aborted':'HALLWAY','preempted':'HALLWAY'})         
            
            ''' Add the hallway subtask(s) '''
            StateMachine.add('HALLWAY', nav_states['hallway'], transitions={'succeeded':'HALLWAY_TASKS','aborted':'','preempted':''})
            
            # When the tasks are done, stop
            StateMachine.add('HALLWAY_TASKS', sm_hallway, transitions={'succeeded':'','aborted':'','preempted':''})         
                        
        # Create and start the SMACH introspection server
        intro_server = IntrospectionServer('clean_house', sm_clean_house, '/SM_ROOT')
        intro_server.start()
        
        # Execute the state machine
        sm_outcome = sm_clean_house.execute()
                
        if len(task_list) > 0:
            message = "Ooops! Not all chores were completed."
            message += "The following rooms need to be revisited: "
            message += str(task_list)
        else:
            message = "All chores complete!"
            
        rospy.loginfo(message)
        easygui.msgbox(message, title="Finished Cleaning")
        
        intro_server.stop()
            
    def move_base_result_cb(self, userdata, status, result):
        if status == actionlib.GoalStatus.SUCCEEDED:
            pass
            
    def cleaning_task_cb(self, userdata):
        rooms_to_clean.remove(userdata.room)
    
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        #sm_nav.request_preempt()
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("House clearning test finished.")
