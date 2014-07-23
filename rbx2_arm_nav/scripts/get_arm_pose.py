#!/usr/bin/env python

"""

Display the current joint positions of the arm in kinematic order of the links

"""

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander

class GetJointStates:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the ROS node
        rospy.init_node('get_joint_states', anonymous=True)
                
        # Initialize the MoveIt! commander for the right arm
        right_arm = MoveGroupCommander('right_arm')
        
        # Get the end-effector link
        end_effector_link = right_arm.get_end_effector_link()
        
        # Joints are stored in the order they appear in the kinematic chain
        joint_names = right_arm.get_active_joints()
        
        joint_names = ['right_arm_shoulder_pan_joint',
                       'right_arm_shoulder_lift_joint',
                       'right_arm_shoulder_roll_joint',
                       'right_arm_elbow_flex_joint',
                       'right_arm_forearm_flex_joint',
                       'right_arm_wrist_flex_joint'
                       ]
        
        # Display the joint names
        rospy.loginfo("Joint names:\n"  + str(joint_names) + "\n")
        
        # Get the current joint angles
        joint_values = right_arm.get_current_joint_values()
        
        # Display the joint values
        rospy.loginfo("Joint values:\n"  + str(joint_values) + "\n")
        
        # Get the end-effector pose
        ee_pose = right_arm.get_current_pose(end_effector_link)
        
        # Display the end-effector pose
        rospy.loginfo("End effector pose:\n" + str(ee_pose))
        
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    GetJointStates()
    