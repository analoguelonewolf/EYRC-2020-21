#!/usr/bin/env python

import rospy
import cv2
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import actionlib
from math import pi
import rospkg
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
from pkg_vb_sim.srv import vacuumGripper,vacuumGripperRequest
from pkg_vb_sim.srv import conveyorBeltPowerMsg,conveyorBeltPowerMsgRequest
import math
from pkg_vb_sim.msg import LogicalCameraImage
from hrwros_gazebo.msg import LogicalCameraImage, Model
import tf2_ros
import tf2_msgs.msg
from pyzbar.pyzbar import decode
from collections import OrderedDict 
import yaml
import os
import copy
import re
import threading
import datetime
import requests
from pkg_task4.msg import msgMqttSub           # Message Class for MQTT Subscription Messages
from pkg_task4.msg import task5
from pkg_task4.msg import msgMqttSubGoal
from pyiot import iot                                   # Custom Python Module to perfrom MQTT Tasks
od = OrderedDict()
item_list_a =[]

class ur5_1(object):
    
    def __init__(self,arg_robot_name):

        rospy.init_node('node_t5_ur5_1', anonymous=True)

        self._robot_ns = '/'  + arg_robot_name
        self._planning_group = "manipulator"
        
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns,wait_for_servers=0)
        self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = 'Box_0'
        self.pkg=""
        self.y_val=0
        self.temp=0
        self._computed_plan = ''
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
        
        self.bridge = CvBridge()
        #rospy.Subscriber("/ros_iot_bridge/mqtt/sub", msgMqttSub, self.func_callback_topic_my_topic, queue_size=12)
        self._handle_goal_pub = rospy.Publisher("/ur5_1/goal/sub", msgMqttSubGoal, queue_size=10)

    def Sort(self,item_list_a): 
        item_list_a.sort(key = lambda x: x[1]) 
        return item_list_a 

    def func_callback_topic_my_topic(self, myMsg):
        global item_list_a
        a=[]
        #rospy.loginfo("Data Received: (%s, %s,%s,%s,%s,%s,%s,%s,%s,%s)", myMsg.topic,myMsg.orderID,myMsg.orderDT,myMsg.item,myMsg.priority,myMsg.quantity,myMsg.city,myMsg.long,myMsg.lat,myMsg.cost)
        if myMsg.priority == 'HP':
	  a.append(1)
	  a.append(myMsg.orderID)
	  a.append(myMsg.city)
	  a.append(myMsg.cost)
	  a.append(myMsg.item)	  
        if myMsg.priority == 'MP':
	  a.append(2)
	  a.append(myMsg.orderID)
	  a.append(myMsg.city)
	  a.append(myMsg.cost)
	  a.append(myMsg.item)
        if myMsg.priority == 'LP':
	  a.append(3)
	  a.append(myMsg.orderID)
	  a.append(myMsg.city)
	  a.append(myMsg.cost)
	  a.append(myMsg.item)
	item_list_a.append(a)
	#del a[:]
        #print item_list

    def goal_pub(self, order_id, city, item):
        msg_goal_pub = msgMqttSubGoal()
        msg_goal_pub.timestamp = rospy.Time.now()
        msg_goal_pub.item = item
        msg_goal_pub.orderID = order_id
        msg_goal_pub.city = city
        self._handle_goal_pub.publish(msg_goal_pub)
	
    def clear_octomap(self):
        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()

    def Conveyer(self,speed):
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        try:
            
            activate=rospy.ServiceProxy('/eyrc/vb/conveyor/set_power',conveyorBeltPowerMsg)
	    activate(speed)
	    rospy.loginfo('\033[94m' + ">>>conveyer belt not in motion" + '\033[0m')

        except rospy.ServiceException:
            print('Service call failed')

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


    def go_to_pose(self, arg_pose):                                  # Function to make manipulator go to a particular position in cordinate space

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

    def func_callback_topic_my_topic_color(self,myMsg):
        global od

        od['packagen00']= myMsg.packagen00
        od['packagen10']= myMsg.packagen10
        od['packagen20']= myMsg.packagen20
        od['packagen30']= myMsg.packagen30

        od['packagen11']= myMsg.packagen11
        od['packagen21']= myMsg.packagen21
        od['packagen31']= myMsg.packagen31

        od['packagen02']= myMsg.packagen02
        od['packagen12']= myMsg.packagen12
        od['packagen22']= myMsg.packagen22
        od['packagen32']= myMsg.packagen32

 
    def for_packagen30(self):
        rospy.logwarn("1. Playing AllZeros to pkgn30 Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'drop_to_packagen30.yaml', 5)
        self.enableGripper()
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'packagen30_to_drop_with_box.yaml', 5)
	self.disableGripper()

    def for_packagen32(self):
        rospy.logwarn("13. Playing drop to pkgn32 Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'drop_to_packagen32.yaml', 5)
        self.enableGripper()
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'packagen32_to_drop_with_box.yaml', 5)
	self.disableGripper()

    def for_packagen22(self):
        rospy.logwarn("10. Playing drop to pkgn22 Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'drop_to_packagen22.yaml', 5)
        self.enableGripper()
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'packagen22_to_drop_with_box.yaml', 5)
	self.disableGripper()

    def for_packagen02(self):
        rospy.logwarn("1. Playing AllZeros to pkgn02 Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'drop_to_packagen02.yaml', 5)
        self.enableGripper()
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'packagen02_to_drop_with_box.yaml', 5)
	self.disableGripper()
        
    def for_packagen00(self):
        rospy.logwarn("2. Playing drop to pkgn00 Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'drop_to_packagen00.yaml', 5)
        self.enableGripper()
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'packagen00_to_drop_with_box.yaml', 5)
	self.disableGripper()

    def for_packagen10(self):
        rospy.logwarn("5. Playing drop to pkgn10 Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'drop_to_packagen10.yaml', 5)
        self.enableGripper()
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'packagen10_to_drop_with_box.yaml', 5)
	self.disableGripper()

    def for_packagen11(self):
        rospy.logwarn("6. Playing drop to pkgn11 Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'drop_to_packagen11.yaml', 5)
        self.enableGripper()
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'packagen11_to_drop_with_box.yaml', 5)
	self.disableGripper()

    def for_packagen12(self):
        rospy.logwarn("7. Playing drop to pkgn12 Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'drop_to_packagen12.yaml', 5)
        self.enableGripper()
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'packagen12_to_drop_with_box.yaml', 5)
	self.disableGripper()

    def for_packagen20(self):
        rospy.logwarn("8. Playing drop to pkgn20 Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'drop_to_packagen20.yaml', 5)
        self.enableGripper()
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'packagen20_to_drop_with_box.yaml', 5)
	self.disableGripper()

    def for_packagen21(self):
        rospy.logwarn("9. Playing drop to pkgn21 Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'drop_to_packagen21.yaml', 5)
        self.enableGripper()
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'packagen21_to_drop_with_box.yaml', 5)
	self.disableGripper()

    def update_list(self):
        ret3 = rospy.wait_for_message("/ros_iot_bridge/mqtt/sub", msgMqttSub)
        while not rospy.is_shutdown():
            #ret3 = rospy.wait_for_message("/ros_iot_bridge/mqtt/sub", msgMqttSub)
            self.func_callback_topic_my_topic(ret3)
            print item_list_a
            self.update_list()


    def update(self, order_id, city, priority, cost, item):  
        now = datetime.datetime.now()
        dt_string = now.strftime("%d/%m/%Y %H:%M:%S")
        parameters = {"id":"OrdersDispatched", "Team Id": "VB#0323","Order ID": order_id,"City" :city, "Item": item, "Priority":priority, "Dispatch Quantity" : '1',"Cost" : cost, "Dispatch Status": "Yes", "Dispatch Date and Time" : dt_string} 
        URL = "https://script.google.com/macros/s/AKfycbwR0GXwHyBSHhAPTfYCvRd2jYcg-UVxJ5_aVyRLMadmvDSOmYfSGy2-Pg/exec"
        response = requests.get(URL, params=parameters)
        print(response.content)

    def update2(self, order_id, city, priority, cost, item):  
        now = datetime.datetime.now()
        dt_string = now.strftime("%d/%m/%Y %H:%M:%S")
        parameters = {"id":"OrdersDispatched", "Team Id": "VB#0323","Order ID": order_id,"City" :city, "Item": item, "Priority":priority, "Dispatch Quantity" : '1',"Cost" : cost, "Dispatch Status": "Yes", "Dispatch Date and Time" : dt_string} 
        URL = "https://script.google.com/macros/s/AKfycbw5xylppoda-8HPjt2Tzq4ShU_Xef-Ik-hEtBPcPk0gdGw8095j4RZ7/exec"
        response = requests.get(URL, params=parameters)
        print(response.content)


    def condition_callback(self, i):
        try:
          self.Sort(item_list_a)
          if item_list_a[i][0] == 1:
            rospy.logwarn(item_list_a)
            if od['packagen00'] == "red":
                self.for_packagen00()
                od['packagen00'] = "empty"
            elif od['packagen02'] == "red":
                self.for_packagen02()
                od['packagen02'] = "empty"
            elif od['packagen10'] == "red":
                self.for_packagen10()
                od['packagen10'] = "empty"
            elif od['packagen11'] == "red":
                self.for_packagen11()
                od['packagen11'] = "empty"
            elif od['packagen12'] == "red":
                self.for_packagen12()
                od['packagen12'] = "empty"
            elif od['packagen20'] == "red":
                self.for_packagen20()
                od['packagen20'] = "empty"
            elif od['packagen21'] == "red":
                self.for_packagen21()
                od['packagen21'] = "empty"
            elif od['packagen22'] == "red":
                self.for_packagen22()
                od['packagen22'] = "empty"
            elif od['packagen30'] == "red":
                self.for_packagen30()
                od['packagen30'] = "empty"
            elif od['packagen32'] == "red":
                self.for_packagen32()
                od['packagen32'] = "empty"
            self.update(item_list_a[i][1],item_list_a[i][2],"HP",item_list_a[i][3],item_list_a[i][4])
            self.update2(item_list_a[i][1],item_list_a[i][2],"HP",item_list_a[i][3],item_list_a[i][4])
	    self.goal_pub(item_list_a[i][1],item_list_a[i][2],item_list_a[i][4])
            item_list_a.pop(i)

          elif item_list_a[i][0] == 2:
            rospy.logwarn(item_list_a)
            if od['packagen00'] == "yellow":
                self.for_packagen00()
                od['packagen00'] = "empty"
            elif od['packagen02'] == "yellow":
                self.for_packagen02()
                od['packagen02'] = "empty"
            elif od['packagen10'] == "yellow":
                self.for_packagen10()
                od['packagen10'] = "empty"
            elif od['packagen11'] == "yellow":
                self.for_packagen11()
                od['packagen11'] = "empty"
            elif od['packagen12'] == "yellow":
                self.for_packagen12()
                od['packagen12'] = "empty"
            elif od['packagen20'] == "yellow":
                self.for_packagen20()
                od['packagen20'] = "empty"
            elif od['packagen21'] == "yellow":
                self.for_packagen21()
                od['packagen21'] = "empty"
            elif od['packagen22'] == "yellow":
                self.for_packagen22()
                od['packagen22'] = "empty"
            elif od['packagen30'] == "yellow":
                self.for_packagen30()
                od['packagen30'] = "empty"
            elif od['packagen32'] == "yellow":
                self.for_packagen32()
                od['packagen32'] = "empty"
            self.update(item_list_a[i][1],item_list_a[i][2],"MP",item_list_a[i][3],item_list_a[i][4])
            #self.update2(item_list_a[i][1],item_list_a[i][2],"HP",item_list_a[i][3],item_list_a[i][4])
	    self.goal_pub(item_list_a[i][1],item_list_a[i][2],item_list_a[i][4])
            item_list_a.pop(i)

          elif item_list_a[i][0] == 3:
            rospy.logwarn(item_list_a)
            if od['packagen00'] == "green":
                self.for_packagen00()
                od['packagen00'] = "empty"
            elif od['packagen02'] == "green":
                self.for_packagen02()
                od['packagen02'] = "empty"
            elif od['packagen10'] == "green":
                self.for_packagen10()
                od['packagen10'] = "empty"
            elif od['packagen11'] == "green":
                self.for_packagen11()
                od['packagen11'] = "empty"
            elif od['packagen12'] == "green":
                self.for_packagen12()
                od['packagen12'] = "empty"
            elif od['packagen20'] == "green":
                self.for_packagen20()
                od['packagen20'] = "empty"
            elif od['packagen21'] == "green":
                self.for_packagen21()
                od['packagen21'] = "empty"
            elif od['packagen22'] == "green":
                self.for_packagen22()
                od['packagen22'] = "empty"
            elif od['packagen30'] == "green":
                self.for_packagen30()
                od['packagen30'] = "empty"
            elif od['packagen32'] == "green":
                self.for_packagen32()
                od['packagen32'] = "empty"
            self.update(item_list_a[i][1],item_list_a[i][2],"LP",item_list_a[i][3],item_list_a[i][4])
            #self.update2(item_list_a[i][1],item_list_a[i][2],"HP",item_list_a[i][3],item_list_a[i][4])
	    self.goal_pub(item_list_a[i][1],item_list_a[i][2],item_list_a[i][4])
            item_list_a.pop(i)

          else:
            self.callback()
        except IndexError:
          rospy.logwarn('waiting for next order')
          self.callback()
 
    def callback(self):
        rospy.logwarn('waiting for next order')
        ret = rospy.wait_for_message("/ros_iot_bridge/mqtt/sub", msgMqttSub)
        
        while not rospy.is_shutdown():
            try:
                #print item_list_a
                t2 = threading.Thread(target=self.condition_callback, args=[0])          
                t2.start()
                t2.join()
            except IndexError:
                rospy.logwarn("index not found")


def main():
  
    ur5=ur5_1('ur5_1')
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'start_to_drop.yaml', 5)
    ur5.Conveyer(100)
    ret2 = rospy.wait_for_message("my_topic", task5)
    ur5.func_callback_topic_my_topic_color(ret2)
    t1 = threading.Thread(target = ur5.update_list)
    t1.start()
    ur5.callback()
   
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

