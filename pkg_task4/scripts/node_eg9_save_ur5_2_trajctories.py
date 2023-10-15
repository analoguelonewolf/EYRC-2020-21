#! /usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg

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

        rospy.init_node('node_moveit_eg6', anonymous=True)

        self._robot_ns = '/'  + arg_robot_name
        self._planning_group = "manipulator"
        
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
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

    def clear_octomap(self):
        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        if (flag_plan == True):
            pass
            # rospy.loginfo(
            #     '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            pass
            # rospy.logerr(
            #     '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):

        number_attempts = 0
        flag_success = False
        
        while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # self.clear_octomap()

    def add_box(self, timeout=4):
        box_name = 'Box_0'
        scene = self._scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.29
        box_pose.pose.position.y = -0.44
        box_pose.pose.position.z = 1.94
        box_name = 'Box_0'
        scene.add_box(box_name, box_pose, size=(0.2, 0.2, 0.2))

    def attach_box(self, timeout=4):
                     
        box_name = 'Box_0'
        robot = self._robot
        scene = self._scene
        eef_link = self._eef_link
        group_names = self._group_names
        grasping_group = 'manipulator'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)

    def enableGripper(self):
        rospy.wait_for_service('/eyrc/vb/ur5_1/activate_vacuum_gripper')
        try:
            
            activate=rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper',vacuumGripper)
	    activate(True)
	    rospy.loginfo('\033[94m' + ">>>vacuum gripper ready" + '\033[0m')

        except rospy.ServiceException:
            print('Service call failed')

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit('ur5_2')

    home = [math.radians(7),
                          math.radians(-140),
                          math.radians(-57),
                          math.radians(-73),
                          math.radians(92),
                          math.radians(-83)]


    #joint angles to bring ur5 arm over Red Container(where red package is to be placed)
    red_bin = [math.radians(-79),
                          math.radians(-112),
                          math.radians(-94),
                          math.radians(-71),
                          math.radians(92),
                          math.radians(-169)]

    #joint angles to bring ur5 arm over green Container(where green package is to be placed)
    yellow_bin = [math.radians(-9),
                          math.radians(-41),
                          math.radians(87),
                          math.radians(-135),
                          math.radians(-89),
                          math.radians(81)]

    #joint angles to bring ur5 arm over blue Container(where blue package is to be placed)
    green_bin = [math.radians(97),
                          math.radians(-105),
                          math.radians(-113),
                          math.radians(-51),
                          math.radians(92),
                          math.radians(7)]





    '''ur5.hard_set_joint_angles(home, 5)

    file_name = 'start_to_home.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )'''


    ur5.hard_set_joint_angles(red_bin, 5)

    file_name = 'home_to_redbin.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )


    '''ur5.hard_set_joint_angles(home, 5)

    file_name = 'redbin_to_home.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )'''


    '''ur5.hard_set_joint_angles(green_bin, 5)

    file_name = 'home_to_greenbin.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )


    ur5.hard_set_joint_angles(home, 5)

    file_name = 'greenbin_to_home.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )


    ur5.hard_set_joint_angles(yellow_bin, 5)

    file_name = 'home_to_yellowbin.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )


    ur5.hard_set_joint_angles(home, 5)

    file_name = 'yellowbin_to_home.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )'''

    
    rospy.sleep(2)

    del ur5



if __name__ == '__main__':
    main()


