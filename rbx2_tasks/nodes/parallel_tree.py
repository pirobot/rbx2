#!/usr/bin/env python

"""
    parallel_tree.py - Version 1.0 2013-09-22
    
    Run two tasks in parallel using the pi_trees library.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.

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

from pi_trees_lib.pi_trees_lib import *
import time

class ParallelExample():
    def __init__(self):
        # The root node
        BEHAVE = Sequence("behave")
        
        # The message to print
        message = "Take me to your leader!"
        
        # How high the counting task should count
        n_count = 10

        # Create a PrintMessage() task as defined later in the script
        PRINT_MESSAGE = PrintMessage("PRINT_MESSAGE", message)
        
        # Create a Count() task, also defined later in the script
        COUNT_TO_10 = Count("COUNT_TO_10", n_count)
        
        # Initialize the ParallelAll task
        PARALLEL_DEMO = ParallelAll("PRINT_AND_COUNT")
        
        # Add the two subtasks to the Parallel task
        PARALLEL_DEMO.add_child(PRINT_MESSAGE)
        PARALLEL_DEMO.add_child(COUNT_TO_10)
        
        # Add the Parallel task to the root task
        BEHAVE.add_child(PARALLEL_DEMO)
        
        # Display the behavior tree
        print "Behavior Tree Structure"
        print_tree(BEHAVE)
        
        # Initialize the overall status
        status = None
            
        # Run the tree
        while not status == TaskStatus.SUCCESS:
            status = BEHAVE.run()
            time.sleep(0.1)

class PrintMessage(Task):
    def __init__(self, name, message, *args, **kwargs):
        super(PrintMessage, self).__init__(name, *args, **kwargs)
        
        self.name = name
        self.message = message
        self.words = message.split()

        print "Creating Print Message task for", self.message
 
    def run(self):
        try:
            word = self.words.pop(0)
            print word,
            time.sleep(0.1)
            if self.words == []:
                return TaskStatus.SUCCESS
            return TaskStatus.RUNNING
        except:
            return TaskStatus.SUCCESS
        
    def reset(self):
        self.words = self.message.split()
        
class Count(Task):
    def __init__(self, name, number, *args, **kwargs):
        super(Count, self).__init__(name, *args, **kwargs)
        
        self.name = name
        self.number = number
        self.count = 0
        print "Creating counting task to", self.number
 
    def run(self):
        if self.count == self.number:
            return TaskStatus.SUCCESS
        else:
            time.sleep(0.1)
            self.count += 1
            print self.count,
            if self.count == self.number:
                return TaskStatus.SUCCESS
            return TaskStatus.RUNNING

    
    def reset(self):
        self.count = 0

if __name__ == '__main__':
    try:
        tree = ParallelExample()
    except KeyboardInterrupt:
        pass
    

