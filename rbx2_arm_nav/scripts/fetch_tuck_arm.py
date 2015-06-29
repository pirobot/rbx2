#!/usr/bin/env python

# Copyright (c) 2013-2014 Unbounded Robotics Inc. 
# All right reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#   * Neither the name of Unbounded Robotics Inc. nor the names of its 
#     contributors may be used to endorse or promote products derived 
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL UNBOUNDED ROBOTICS INC. BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
Usage: tuck_arm.py
  Tucks the robot arm (does not perform collision avoidance)
"""

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

class TuckArm:
    joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                   "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    tucked  = [-1.3901, 1.3439, -2.8327, -1.8119, 0.0, -1.6571, 0.0]

    def __init__(self):
        rospy.loginfo("Waiting for arm_controller...")
        self.client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        self.client.wait_for_server()
        rospy.loginfo("...connected.")

        self.state_recv = False
        self.sub = rospy.Subscriber("joint_states", JointState, self.state_callback)

    def state_callback(self, msg):
        self.state_recv = True

    def tuck_arm(self):
        while not self.state_recv:
            rospy.loginfo("Waiting for controllers to be up...")
            rospy.sleep(0.1)

        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = self.tucked
        trajectory.points[0].velocities = [0.0 for i in self.joint_names]
        trajectory.points[0].accelerations = [0.0 for i in self.joint_names]
        trajectory.points[0].time_from_start = rospy.Duration(5.0)

        rospy.loginfo("Tucking arm...")
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = rospy.Duration(0.0)

        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(6.0))
        rospy.loginfo("...done")

if __name__ == "__main__":
    rospy.init_node("tuck_my_arm")
    t = TuckArm()
    t.tuck_arm()
