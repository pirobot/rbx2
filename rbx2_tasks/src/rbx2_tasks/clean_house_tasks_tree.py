#!/usr/bin/env python

""" clean_house_tasks_tree.py - Version 1.0 2013-12-20

    Create a number of simulated cleaning tasks.

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
from pi_trees_lib.pi_trees_lib import *
from geometry_msgs.msg import Twist

class Vacuum(Task):
    def __init__(self, room=None, timer=3, *args):
        name = "VACUUM_" + room.upper()
        super(Vacuum, self).__init__(name)    
        self.name = name
        self.room = room
        self.counter = timer
        self.finished = False
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.cmd_vel_msg = Twist()
        self.cmd_vel_msg.linear.x = 0.05

    def run(self):
        if self.finished:
            return TaskStatus.SUCCESS
        else:
            rospy.loginfo('Vacuuming the floor in the ' + str(self.room))
            
            while self.counter > 0:
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
                self.cmd_vel_msg.linear.x *= -1
                rospy.loginfo(self.counter)
                self.counter -= 1
                rospy.sleep(1)
                return TaskStatus.RUNNING
            
            self.finished = True
            self.cmd_vel_pub.publish(Twist())
            message = "Finished vacuuming the " + str(self.room) + "!"
            rospy.loginfo(message)

    
class Mop(Task):
    def __init__(self, room=None, timer=3, *args):
        name = "MOP_" + room.upper()
        super(Mop, self).__init__(name)
        self.name = name
        self.room = room
        self.counter = timer
        self.finished = False
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.cmd_vel_msg = Twist()
        self.cmd_vel_msg.linear.x = 0.05
        self.cmd_vel_msg.angular.z = 1.2

    def run(self):
        if self.finished:
            return TaskStatus.SUCCESS
        else:
            rospy.loginfo('Mopping the floor in the ' + str(self.room))
            
            while self.counter > 0:
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
                self.cmd_vel_msg.linear.x *= -1
                rospy.loginfo(self.counter)
                self.counter -= 1
                rospy.sleep(1)
                return TaskStatus.RUNNING
            
            self.finished = True
            self.cmd_vel_pub.publish(Twist())
            message = "Done mopping the " + str(self.room) + "!"
            rospy.loginfo(message)            

class Scrub(Task):
    def __init__(self, room=None, timer=7, *args):
        name = "SCRUB_" + room.upper()
        super(Scrub, self).__init__(name)    
        self.name = name
        self.room = room
        self.finished = False
        self.counter = timer
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.cmd_vel_msg = Twist()
        self.cmd_vel_msg.linear.x = 0.3
        self.cmd_vel_msg.angular.z = 0.2

    def run(self):
        if self.finished:
            return TaskStatus.SUCCESS
        else:
            rospy.loginfo('Cleaning the tub...')
            
            while self.counter > 0:
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
                self.cmd_vel_msg.linear.x *= -1
                if self.counter % 2 == 5:
                    self.cmd_vel_msg.angular.z *= -1
                rospy.loginfo(self.counter)
                self.counter -= 1
                rospy.sleep(0.2)
                return TaskStatus.RUNNING
        
            self.finished = True
            self.cmd_vel_pub.publish(Twist())
            message = "The tub is clean!"
            rospy.loginfo(message)

            