#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Feb 28 08:14:25 2022

@author: Raj Kumar Gupta

This node is used to move the OW_arm42v3 from point A to pointB by setting the 
joints of the arm"""

#Import the required python modules
import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import actionlib
import math


class ArmMoveit:
    """Class to send joints goal to the arm"""
    # Constructor
    def __init__(self):

        rospy.init_node('node_set_joint_angles', anonymous=True)

        self._planning_group = "planning_group_arm"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._group_names = self._robot.get_group_names()


        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def set_joint_angles(self, arg_list_joint_angles):
        """Function to set the joint angles"""
        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        flag_plan  = self._exectute_trajectory_client.wait_for_result()
        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan    

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')
    
def main():
    """Main Function"""
    
    #wait for rviz to up
    rospy.sleep(4)
    
    #moveit arm instance
    arm = ArmMoveit()
    
    #joint angle one
    lst_joint_angles_1 = [math.radians(0),
                          math.radians(-80),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0)]
    
    #joint angle two
    lst_joint_angles_2 = [math.radians(101),
                          math.radians(62),
                          math.radians(-9),
                          math.radians(0),
                          math.radians(-53),
                          math.radians(-101)]
    
    #set joint angle one
    arm.set_joint_angles(lst_joint_angles_1)
    
    #sleep for ten seconds
    rospy.sleep(10)
    
    #set joint angle two
    arm.set_joint_angles(lst_joint_angles_2)
    
    del arm 
    
if __name__ == "__main__":
    main()
    
    
