#! /usr/bin/env python

import os
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
from moveit_ros_planning_interface import _moveit_move_group_interface
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('node_t2_ur5_1_pick_place.py', anonymous=True)

        # self._g.set_goal_tolerance(0.01)
        # self._g = _moveit_move_group_interface.MoveGroupInterface("ur5_1_planning_group", "robot_description", "", 5.0)
        self._planning_group = "ur5_1_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()


        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def go_to_predefined_pose(self, arg_pose_name):
        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')
    
    def add_box(self, timeout=4):
      box_name = 'Box_0'
      scene = self._scene
      box_pose = geometry_msgs.msg.PoseStamped()
      box_pose.header.frame_id = "world"
      box_pose.pose.orientation.w = 1.0
      box_pose.pose.position.x = 0.01
      box_pose.pose.position.y = 0.45
      box_pose.pose.position.z = 1.87
      box_name = 'Box_0'
      scene.add_box(box_name, box_pose, size=(0.2, 0.2, 0.2))

    def go_to_pose(self, arg_pose):
        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

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

    def attach_box(self, timeout=4):
      box_name = 'Box_0'
      robot = self._robot
      scene = self._scene
      eef_link = self._eef_link
      group_names = self._group_names

      grasping_group = 'ur5_1_planning_group'
      touch_links = robot.get_link_names(group=grasping_group)
      scene.attach_box(eef_link, box_name, touch_links=touch_links)
    
    #   return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()

    #placing box
    ur5.add_box()
    rospy.sleep(2)
    
    #planning EE to box
    lst_joint_angles_1 = [math.radians(66),
                          math.radians(-64),
                          math.radians(-23),
                          math.radians(-93),
                          math.radians(-66),
                          math.radians(-180)]
    while not rospy.is_shutdown():
        flag_plan = ur5.set_joint_angles(lst_joint_angles_1)
        rospy.sleep(2)
        if flag_plan == True:
            break
        
    # activating gripper    
    ur5.attach_box()
    os.popen('rosservice call /eyrc/vb/ur5_1/activate_vacuum_gripper "activate_vacuum_gripper: true"')

    #planning EE to bin
    lst_joint_angles_1 = [math.radians(34),
                          math.radians(-126),
                          math.radians(-64),
                          math.radians(-106),
                          math.radians(-18),
                          math.radians(134)]
    while not rospy.is_shutdown():
        flag_plan = ur5.set_joint_angles(lst_joint_angles_1)
        rospy.sleep(2)
        if flag_plan == True:
            break

    # deactivating gripper    
    os.popen('rosservice call /eyrc/vb/ur5_1/activate_vacuum_gripper "activate_vacuum_gripper: false"')    
    
    #planning EE back to AllZeros state
    ur5.go_to_predefined_pose('allZeros')
    
    del ur5


if __name__ == '__main__':
    main()


