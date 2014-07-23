#!/usr/bin/env python

""" monitor_fake_battery.py - Version 1.0 2013-04-21

    Monitor and recharge a simulated battery using SMACH states

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
from smach import State, StateMachine
from smach_ros import MonitorState, ServiceState, IntrospectionServer
from rbx2_msgs.srv import *
from std_msgs.msg import Float32

class main():
    def __init__(self):
        rospy.init_node('monitor_fake_battery', anonymous=False)
        
        rospy.on_shutdown(self.shutdown)
        
        # Set the low battery threshold (between 0 and 100)
        self.low_battery_threshold = rospy.get_param('~low_battery_threshold', 50)
 
        # Initialize the state machine
        sm_battery_monitor = StateMachine(outcomes=[])

        with sm_battery_monitor:
            # Add a MonitorState to subscribe to the battery level topic
            StateMachine.add('MONITOR_BATTERY',
                 MonitorState('battery_level', Float32, self.battery_cb), 
                 transitions={'invalid':'RECHARGE_BATTERY',
                              'valid':'MONITOR_BATTERY',
                              'preempted':'MONITOR_BATTERY'},)

            # Add a ServiceState to simulate a recharge using the set_battery_level service
            StateMachine.add('RECHARGE_BATTERY',
                 ServiceState('battery_simulator/set_battery_level', SetBatteryLevel, request=100), 
                 transitions={'succeeded':'MONITOR_BATTERY',
                              'aborted':'MONITOR_BATTERY',
                              'preempted':'MONITOR_BATTERY'})
            
        # Create and start the SMACH introspection server
        intro_server = IntrospectionServer('monitor_battery', sm_battery_monitor, '/SM_ROOT')
        intro_server.start()
        
        # Execute the state machine
        sm_outcome = sm_battery_monitor.execute()
                        
        intro_server.stop()
        
    def battery_cb(self, userdata, msg):
        rospy.loginfo("Battery Level: " + str(msg))
        if msg.data < self.low_battery_threshold:
            return False
        else:
            return True

    def shutdown(self):
        rospy.loginfo("Stopping the battery monitor...")
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Battery monitor finished.")
