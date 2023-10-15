#! /usr/bin/env python
#importing directories
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg
from pkg_vb_sim.msg import Model 
from pkg_vb_sim.msg import LogicalCameraImage 
from pkg_vb_sim.srv import ConveyorBeltControl
from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.srv import vacuumGripper
import tf2_ros
import tf2_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import requests

import yaml
import os
import math
import time
import sys
import copy

from std_srvs.srv import Empty

class Ur5Moveit:

    # Constructor
    def __init__(self, arg_robot_name):

        rospy.init_node('node_moveit_ur5_1', anonymous=True)

        self._robot_ns = '/'  + arg_robot_name
        self._planning_group = "manipulator"
        
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
	self._group.set_planning_time(20)
        self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''


        # Attribute to store computed trajectory by the planner	
        self._computed_plan = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')


        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task4')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        rospy.loginfo( "Package Path: {}".format(self._file_path) )


        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    #function to clear the octomap
    def clear_octomap(self):
        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()


    #functions to play saved trajectories
    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        file_path = arg_file_path + arg_file_name
        
        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)
        
        ret = self._group.execute(loaded_plan)
        return ret

    
    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
        number_attempts = 0
        flag_success = False

        while ( (number_attempts <= arg_max_attempts) and (flag_success is False) ):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            #self.clear_octomap()
        return True


    #functions to control gripper
    def enableGripper(self): #to enable gripper
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
        try:
            
            activate=rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1',vacuumGripper)
	    activate(True)
	    rospy.loginfo('\033[94m' + ">>>vacuum gripper ready" + '\033[0m')

        except rospy.ServiceException:
            print('Service call failed')    

    def disableGripper(self):  #to disable gripper
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
        try:
            
            activate=rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1',vacuumGripper)
	    activate(False)
	    rospy.loginfo('\033[94m' + ">>>vacuum gripper ready" + '\033[0m')

        except rospy.ServiceException:
            print('Service call failed')

    #rosservice function to power conveyer belt
    def Conveyer(self,speed):
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        try:
            
            activate=rospy.ServiceProxy('/eyrc/vb/conveyor/set_power',conveyorBeltPowerMsg)
	    activate(speed)
	    rospy.loginfo('\033[94m' + ">>>conveyer belt not in motion" + '\033[0m')

        except rospy.ServiceException:
            print('Service call failed')

        
        return True

        
    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit('ur5_1') #class callback

    ur5.Conveyer(100)
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'start_to_drop.yaml', 5)
    while not rospy.is_shutdown():

        #playing saved trajectories

        rospy.logwarn("1. Playing AllZeros to pkgn02 Trajectory File")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'drop_to_packagen02.yaml', 5)
        ur5.enableGripper()
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'packagen02_to_drop_with_box.yaml', 5)
	ur5.disableGripper()

        rospy.logwarn("2. Playing drop to pkgn00 Trajectory File")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'drop_to_packagen00.yaml', 5)
        ur5.enableGripper()
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'packagen00_to_drop_with_box.yaml', 5)
	ur5.disableGripper()


        rospy.logwarn("5. Playing drop to pkgn10 Trajectory File")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'drop_to_packagen10.yaml', 5)
        ur5.enableGripper()
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'packagen10_to_drop_with_box.yaml', 5)
	ur5.disableGripper()

        rospy.logwarn("6. Playing drop to pkgn11 Trajectory File")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'drop_to_packagen11.yaml', 5)
        ur5.enableGripper()
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'packagen11_to_drop_with_box.yaml', 5)
	ur5.disableGripper()

        rospy.logwarn("7. Playing drop to pkgn12 Trajectory File")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'drop_to_packagen12.yaml', 5)
        ur5.enableGripper()
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'packagen12_to_drop_with_box.yaml', 5)
	ur5.disableGripper()

        rospy.logwarn("8. Playing drop to pkgn20 Trajectory File")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'drop_to_packagen20.yaml', 5)
        ur5.enableGripper()
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'packagen20_to_drop_with_box.yaml', 5)
	ur5.disableGripper()

        rospy.logwarn("9. Playing drop to pkgn21 Trajectory File")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'drop_to_packagen21.yaml', 5)
        ur5.enableGripper()
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'packagen21_to_drop_with_box.yaml', 5)
	ur5.disableGripper()

        rospy.logwarn("1. Playing AllZeros to pkgn30 Trajectory File")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'drop_to_packagen30.yaml', 5)
        ur5.enableGripper()
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'packagen30_to_drop_with_box.yaml', 5)
	ur5.disableGripper()

        rospy.logwarn("13. Playing drop to pkgn32 Trajectory File")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'drop_to_packagen32.yaml', 5)
        ur5.enableGripper()
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'packagen32_to_drop_with_box.yaml', 5)
	ur5.disableGripper()

        rospy.logwarn("10. Playing drop to pkgn22 Trajectory File")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'drop_to_packagen22.yaml', 5)
        ur5.enableGripper()
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'packagen22_to_drop_with_box.yaml', 5)
	ur5.disableGripper()
        
	rospy.spin()


    del ur5



if __name__ == '__main__':
    main()


