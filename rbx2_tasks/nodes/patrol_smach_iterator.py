#!/usr/bin/env python

""" patrol_smach_iterator.py - Version 1.0 2013-10-23

    Control a robot using SMACH to patrol a square area a specified number of times

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
from smach import StateMachine, Iterator
from smach_ros import SimpleActionState, IntrospectionServer
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi

class main():
    def __init__(self):
        rospy.init_node('patrol_smach', anonymous=False)
        
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # Initialize a number of parameters and variables
        self.init()
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait up to 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        
        rospy.loginfo("Connected to move_base action server")
        
        # Track success rate of getting to the goal locations
        self.n_succeeded = 0
        self.n_aborted = 0
        self.n_preempted = 0
        self.n_patrols = 2
        
        # Turn the waypoints into SMACH states
        nav_states = list()
        
        for waypoint in self.waypoints:           
            nav_goal = MoveBaseGoal()
            nav_goal.target_pose.header.frame_id = 'map'
            nav_goal.target_pose.pose = waypoint
            move_base_state = SimpleActionState('move_base', MoveBaseAction, goal=nav_goal, result_cb=self.move_base_result_cb,
                                                 exec_timeout=rospy.Duration(10.0),
                                                 server_wait_timeout=rospy.Duration(10.0))
            nav_states.append(move_base_state)
            
            move_base_state = SimpleActionState('move_base', MoveBaseAction, goal=nav_goal, result_cb=self.move_base_result_cb,
                                                 exec_timeout=rospy.Duration(10.0))
            
        # Initialize the top level state machine
        self.sm = StateMachine(outcomes=['succeeded','aborted','preempted'])
        
        with self.sm:
            # Initialize the iterator
            self.sm_patrol_iterator = Iterator(outcomes = ['succeeded','preempted','aborted'],
                                          input_keys = [],
                                          it = lambda: range(0, self.n_patrols),
                                          output_keys = [],
                                          it_label = 'index',
                                          exhausted_outcome = 'succeeded')
            
            with self.sm_patrol_iterator:
                # Initialize the patrol state machine
                self.sm_patrol = StateMachine(outcomes=['succeeded','aborted','preempted','continue'])
        
                # Add the states to the state machine with the appropriate transitions
                with self.sm_patrol:            
                    StateMachine.add('NAV_STATE_0', nav_states[0], transitions={'succeeded':'NAV_STATE_1','aborted':'NAV_STATE_1','preempted':'NAV_STATE_1'})
                    StateMachine.add('NAV_STATE_1', nav_states[1], transitions={'succeeded':'NAV_STATE_2','aborted':'NAV_STATE_2','preempted':'NAV_STATE_2'})
                    StateMachine.add('NAV_STATE_2', nav_states[2], transitions={'succeeded':'NAV_STATE_3','aborted':'NAV_STATE_3','preempted':'NAV_STATE_3'})
                    StateMachine.add('NAV_STATE_3', nav_states[3], transitions={'succeeded':'NAV_STATE_4','aborted':'NAV_STATE_4','preempted':'NAV_STATE_4'})
                    StateMachine.add('NAV_STATE_4', nav_states[0], transitions={'succeeded':'continue','aborted':'continue','preempted':'continue'})
                
                # Close the sm_patrol machine and add it to the iterator
                Iterator.set_contained_state('PATROL_STATE', self.sm_patrol, loop_outcomes=['continue'])
            
            # Close the top level state machine 
            StateMachine.add('PATROL_ITERATOR', self.sm_patrol_iterator, {'succeeded':'succeeded', 'aborted':'aborted'})
            
        # Create and start the SMACH introspection server
        intro_server = IntrospectionServer('patrol', self.sm, '/SM_ROOT')
        intro_server.start()
        
        # Execute the state machine
        sm_outcome = self.sm.execute()
        
        rospy.loginfo('State Machine Outcome: ' + str(sm_outcome))
                
        intro_server.stop()
        
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
        
    def init(self):
        # How big is the square we want the robot to patrol?
        self.square_size = rospy.get_param("~square_size", 1.0) # meters
        
        # How many times should we execute the patrol loop
        self.n_patrols = rospy.get_param("~n_patrols", 3) # meters
        
        # Create a list to hold the target quaternions (orientations)
        quaternions = list()
        
        # First define the corner orientations as Euler angles
        euler_angles = (pi/2, pi, 3*pi/2, 0)
        
        # Then convert the angles to quaternions
        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            q = Quaternion(*q_angle)
            quaternions.append(q)
        
        # Create a list to hold the waypoint poses
        self.waypoints = list()
                
        # Append each of the four waypoints to the list.  Each waypoint
        # is a pose consisting of a position and orientation in the map frame.
        self.waypoints.append(Pose(Point(0.0, 0.0, 0.0), quaternions[3]))
        self.waypoints.append(Pose(Point(self.square_size, 0.0, 0.0), quaternions[0]))
        self.waypoints.append(Pose(Point(self.square_size, self.square_size, 0.0), quaternions[1]))
        self.waypoints.append(Pose(Point(0.0, self.square_size, 0.0), quaternions[2]))
        
        # Initialize the waypoint visualization markers for RViz
        self.init_waypoint_markers()
        
        # Set a visualization marker at each waypoint        
        for waypoint in self.waypoints:           
            p = Point()
            p = waypoint.position
            self.waypoint_markers.points.append(p)
            
        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
        
        rospy.loginfo("Starting SMACH test")
        
        # Publish the waypoint markers
        self.marker_pub.publish(self.waypoint_markers)
        rospy.sleep(1)
        self.marker_pub.publish(self.waypoint_markers)

    def init_waypoint_markers(self):
        # Set up our waypoint markers
        marker_scale = 0.2
        marker_lifetime = 0 # 0 is forever
        marker_ns = 'waypoints'
        marker_id = 0
        marker_color = {'r': 1.0, 'g': 0.7, 'b': 1.0, 'a': 1.0}
        
        # Define a marker publisher.
        self.marker_pub = rospy.Publisher('waypoint_markers', Marker)
        
        # Initialize the marker points list.
        self.waypoint_markers = Marker()
        self.waypoint_markers.ns = marker_ns
        self.waypoint_markers.id = marker_id
        self.waypoint_markers.type = Marker.CUBE_LIST
        self.waypoint_markers.action = Marker.ADD
        self.waypoint_markers.lifetime = rospy.Duration(marker_lifetime)
        self.waypoint_markers.scale.x = marker_scale
        self.waypoint_markers.scale.y = marker_scale
        self.waypoint_markers.color.r = marker_color['r']
        self.waypoint_markers.color.g = marker_color['g']
        self.waypoint_markers.color.b = marker_color['b']
        self.waypoint_markers.color.a = marker_color['a']
        
        self.waypoint_markers.header.frame_id = 'odom'
        self.waypoint_markers.header.stamp = rospy.Time.now()
        self.waypoint_markers.points = list()

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.sm_patrol.request_preempt()
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("SMACH test finished.")
