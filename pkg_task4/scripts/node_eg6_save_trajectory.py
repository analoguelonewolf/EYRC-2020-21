#! /usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg
from pkg_vb_sim.srv import vacuumGripper,vacuumGripperRequest
import yaml
import os
import math
import time
import sys
import copy

from std_srvs.srv import Empty

'''0.30,-0.42,1.92,0.02,0.69,1.46'''

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
	self._group.set_planning_time(90)
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
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
        try:
            
            activate=rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1',vacuumGripper)
	    activate(True)
	    rospy.loginfo('\033[94m' + ">>>vacuum gripper ready" + '\033[0m')

        except rospy.ServiceException:
            print('Service call failed')

    def disableGripper(self):
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
        try:
            
            activate=rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1',vacuumGripper)
	    activate(False)
	    rospy.loginfo('\033[94m' + ">>>vacuum gripper ready" + '\033[0m')

        except rospy.ServiceException:
            print('Service call failed')

    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        file_path = arg_file_path + arg_file_name
        
        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)
        
        ret = self._group.execute(loaded_plan)
        # rospy.logerr(ret)
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

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit('ur5_1')

    start = [math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0)]

    packagen00 = [math.radians(-54),
                          math.radians(-73),
                          math.radians(15),
                          math.radians(-121),
                          math.radians(-129),
                          math.radians(180)]

    packagen01 = [math.radians(129),
                          math.radians(-86),
                          math.radians(-37),
                          math.radians(-56),
                          math.radians(49),
                          math.radians(180)]

    packagen02 = [math.radians(59),
                          math.radians(-110),
                          math.radians(-6),
                          math.radians(-64),
                          math.radians(119),
                          math.radians(180)]

    packagen10 = [math.radians(-54),
                          math.radians(-97),
                          math.radians(84),
                          math.radians(-166),
                          math.radians(-129),
                          math.radians(180)]

    packagen11 = [math.radians(119),
                          math.radians(-63),
                          math.radians(-102),
                          math.radians(-15),
                          math.radians(58),
                          math.radians(180)]

    packagen12 = [math.radians(56),
                          math.radians(-82),
                          math.radians(-84),
                          math.radians(-14),
                          math.radians(122),
                          math.radians(180)]

    packagen20 = [math.radians(-54),
                          math.radians(-98),
                          math.radians(88),
                          math.radians(10),
                          math.radians(129),
                          math.radians(0)]

    packagen21 = [math.radians(123),
                          math.radians(-61),
                          math.radians(-104),
                          math.radians(166),
                          math.radians(-55),
                          math.radians(0)]

    packagen22 = [math.radians(58),
                          math.radians(-84),
                          math.radians(-117),
                          math.radians(21),
                          math.radians(120),
                          math.radians(180)]

    packagen30 = [math.radians(-56),
                          math.radians(-95),
                          math.radians(116),
                          math.radians(-21),
                          math.radians(125),
                          math.radians(0)]

    packagen31 = [math.radians(-119),
                          math.radians(-118),
                          math.radians(134),
                          math.radians(-16),
                          math.radians(69),
                          math.radians(-179)]

    packagen32 = [math.radians(57),
                          math.radians(-106),
                          math.radians(-133),
                          math.radians(58),
                          math.radians(121),
                          math.radians(180)]

    drop = [math.radians(164),
                          math.radians(-14),
                          math.radians(15),
                          math.radians(-8),
                          math.radians(-16),
                          math.radians(-83)]



    # 1. Save AllZeros to Pose#1 Trajectory stored in "_computed_plan" attribute in a File
    # The "_computed_plan" attribute is set in method hard_set_joint_angles()
    '''ur5.hard_set_joint_angles(packagen02, 5)

    file_name = 'start_to_packagen02.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )

    ur5.set_joint_angles(drop)'''


    # 2. Save Pose#1 to Pose#2 Trajectory stored in "_computed_plan" attribute in a File
    # The "_computed_plan" attribute is set in method hard_set_joint_angles()
    '''ur5.hard_set_joint_angles(packagen02, 5)

    file_name = 'drop_to_packagen02.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )

    ur5.set_joint_angles(drop)'''

    # 3. Save Pose#2 to Pose#3 Trajectory stored in "_computed_plan" attribute in a File
    # The "_computed_plan" attribute is set in method hard_set_joint_angles()
    '''ur5.hard_set_joint_angles(packagen01, 5)

    file_name = 'drop_to_packagen01.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )

    ur5.set_joint_angles(drop)


    # 4. Save Pose#3 to AllZeros Trajectory stored in "_computed_plan" attribute in a File
    # The "_computed_plan" attribute is set in method hard_set_joint_angles()
    ur5.hard_set_joint_angles(packagen00, 5)

    file_name = 'drop_to_packagen00.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )

    ur5.set_joint_angles(drop)
    
    # 4. Save Pose#3 to AllZeros Trajectory stored in "_computed_plan" attribute in a File
    # The "_computed_plan" attribute is set in method hard_set_joint_angles()
    ur5.hard_set_joint_angles(packagen10, 5)

    file_name = 'drop_to_packagen10.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    ur5.set_joint_angles(drop)

    # 4. Save Pose#3 to AllZeros Trajectory stored in "_computed_plan" attribute in a File
    # The "_computed_plan" attribute is set in method hard_set_joint_angles()
    ur5.hard_set_joint_angles(packagen11, 5)

    file_name = 'drop_to_packagen11.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    ur5.set_joint_angles(drop)

    # 4. Save Pose#3 to AllZeros Trajectory stored in "_computed_plan" attribute in a File
    # The "_computed_plan" attribute is set in method hard_set_joint_angles()
    ur5.hard_set_joint_angles(packagen12, 5)

    file_name = 'drop_to_packagen12.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    ur5.set_joint_angles(drop)

    # 4. Save Pose#3 to AllZeros Trajectory stored in "_computed_plan" attribute in a File
    # The "_computed_plan" attribute is set in method hard_set_joint_angles()
    ur5.hard_set_joint_angles(packagen20, 5)

    file_name = 'drop_to_packagen20.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    ur5.set_joint_angles(drop)'''

    # 4. Save Pose#3 to AllZeros Trajectory stored in "_computed_plan" attribute in a File
    # The "_computed_plan" attribute is set in method hard_set_joint_angles()
    
    '''ur5.hard_set_joint_angles(packagen21, 5)

    file_name = 'drop_to_packagen21.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    ur5.set_joint_angles(drop)'''

    # 4. Save Pose#3 to AllZeros Trajectory stored in "_computed_plan" attribute in a File
    # The "_computed_plan" attribute is set in method hard_set_joint_angles()
    '''ur5.set_joint_angles(drop)
    ur5.hard_set_joint_angles(packagen22, 5)

    file_name = 'drop_to_packagen22.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )'''


    # 4. Save Pose#3 to AllZeros Trajectory stored in "_computed_plan" attribute in a File
    # The "_computed_plan" attribute is set in method hard_set_joint_angles()
    '''ur5.hard_set_joint_angles(packagen30, 5)

    file_name = 'drop_to_packagen30.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    ur5.set_joint_angles(drop)'''

    # 4. Save Pose#3 to AllZeros Trajectory stored in "_computed_plan" attribute in a File
    # The "_computed_plan" attribute is set in method hard_set_joint_angles()
    '''ur5.hard_set_joint_angles(packagen31, 5)

    file_name = 'drop_to_packagen31.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    ur5.set_joint_angles(drop)'''

    # 4. Save Pose#3 to AllZeros Trajectory stored in "_computed_plan" attribute in a File
    # The "_computed_plan" attribute is set in method hard_set_joint_angles()
    '''ur5.hard_set_joint_angles(packagen32, 5)

    file_name = 'drop_to_packagen32.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    ur5.set_joint_angles(drop)'''

    '''ur5.hard_set_joint_angles(packagen30, 5)
    file_name = 'start_to_packagen30.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    #ur5.set_joint_angles(drop)'''

    '''ur5.set_joint_angles(drop)
    rospy.logwarn("1. Playing AllZeros to pkgn30 Trajectory File")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'drop_to_packagen30.yaml', 5)'''

    #rospy.logwarn("1. Playing AllZeros to pkgn30 Trajectory File")
    #ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'packagen30_to_drop_wb.yaml', 5)

    '''ur5.hard_set_joint_angles(drop, 5)

    file_name = 'packagen30_to_drop_wb.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )'''

    #ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'packagen30_to_drop_wb.yaml', 5)
    #ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'packagen30_to_drop_wb.yaml', 5)
    '''ur5.set_joint_angles(drop)
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'drop_to_packagen01.yaml', 5)'''
    ur5.enableGripper()
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'packagen01_to_drop_with_box.yaml', 5)
    ur5.disableGripper()
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'drop_to_packagen02.yaml', 5)
    ur5.enableGripper()
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'packagen02_to_drop_with_box.yaml', 5)
    ur5.disableGripper()
    rospy.sleep(2)

    del ur5



if __name__ == '__main__':
    main()


