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
import datetime
import requests
import threading
from pkg_task4.msg import task5
from pkg_task4.msg import msgMqttSubGoal
item_list_a =[]
item_list_b =[]
item_list_c=[]
od=OrderedDict()
class TF:
    def __init__(self):
         
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)

    def func_tf_print(self, arg_frame_1, arg_frame_2):
        try:#subscribing to the logical camera 
            trans = self._tfBuffer.lookup_transform(arg_frame_1, arg_frame_2, rospy.Time())
            #saving coordinates in p,q,r
            p=format(trans.transform.translation.x) 
            q=format(trans.transform.translation.y) 
            r=format(trans.transform.translation.z) 
            return p,q,r
               
            '''x: {} \n.format(trans.transform.rotation.x) +
             .format(trans.transform.rotation.w) )'''

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("TF error")

    #function to callback package names
    def callback_image(self,myMsg):
            try:
                models = []
                models.append(myMsg.models.pop())
            
            except Exception as e:
                rospy.loginfo("model not received!!!! ",e.__class__)                    

            else:
                self.model = models.pop()   #model_type, pose 
class Camera1(object):
    
    def __init__(self,arg_robot_name):

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
        rospy.Subscriber("my_topic", task5, self.func_callback_topic_my_topic, queue_size=12)
        #rospy.Subscriber("/ros_iot_bridge/mqtt/sub", msgMqttSub, self.func_callback_items, queue_size=12)

    def func_callback_items(self, myMsg):
        global item_list_a, item_list_b
        item_list_a.append(myMsg.orderID)
        item_list_b.append(myMsg.city)
        item_list_c.append(myMsg.item)

    def func_callback_topic_my_topic(self,myMsg2):
        global od

        od['packagen00']= myMsg2.packagen00
        od['packagen10']= myMsg2.packagen10
        od['packagen20']= myMsg2.packagen20
        od['packagen30']= myMsg2.packagen30

        od['packagen01']= myMsg2.packagen01
        od['packagen11']= myMsg2.packagen11
        od['packagen21']= myMsg2.packagen21
        od['packagen31']= myMsg2.packagen31

        od['packagen02']= myMsg2.packagen02
        od['packagen12']= myMsg2.packagen12
        od['packagen22']= myMsg2.packagen22
        od['packagen32']= myMsg2.packagen32

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
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
        try:
            
            activate=rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2',vacuumGripper)
	    activate(True)
	    rospy.loginfo('\033[94m' + ">>>vacuum gripper ready" + '\033[0m')

        except rospy.ServiceException:
            print('Service call failed')    
    def disableGripper(self):
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
        try:
            
            activate=rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2',vacuumGripper)
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


    def go_to_redbin_back(self,index):
	rospy.logwarn("10. Playing drop to redbin Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'home_to_redbin.yaml', 5)
	self.disableGripper()
        #self.update(item_list_a[index], item_list_b[index],item_list_c[index])
        #self.update2(item_list_a[index], item_list_b[index],item_list_c[index])
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'redbin_to_home.yaml', 5)

    def go_to_yellowbin_back(self,index):
	rospy.logwarn("10. Playing drop to yellowbin Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'home_to_yellowbin.yaml', 5)
	self.disableGripper()
        #self.update(item_list_a[index], item_list_b[index],item_list_c[index])
        #self.update2(item_list_a[index], item_list_b[index],item_list_c[index])
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'yellowbin_to_home.yaml', 5)

    def go_to_greenbin_back(self,index):
	rospy.logwarn("10. Playing drop to greenbin Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'home_to_greenbin.yaml', 5)
	self.disableGripper()
        #self.update(item_list_a[index], item_list_b[index],item_list_c[index])
        #self.update2(item_list_a[index], item_list_b[index],item_list_c[index])
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'greenbin_to_home.yaml', 5)


    def update_list(self):
        ret3 = rospy.wait_for_message("/ur5_1/goal/sub", msgMqttSubGoal)
        while not rospy.is_shutdown():
            #ret3 = rospy.wait_for_message("/ros_iot_bridge/mqtt/sub", msgMqttSub)
            self.func_callback_items(ret3)
            self.update_list()

    def update(self, order_id, city, item):   
        now = datetime.datetime.now()
        date = now.strftime("%d/%m/%y %H:%M:%S")

        NextDay_Date = datetime.datetime.today() + datetime.timedelta(days=1)
        etd = NextDay_Date.strftime("%d/%m/%Y %H:%M:%S")
        if item == 'Medicine':
          priority='HP'
          cost = '450'
        elif item == 'Food':
          priority='MP'
          cost = '250'
        elif item == 'Clothes':
          priority='LP'
          cost = '150'
        parameters = {"id":"OrdersShipped","Team Id": "VB#0323","Order ID": order_id,"City" :city, "Item": item, "Priority":priority, "Shipped Quantity" : '1',"Cost" : cost, "Shipped Status": "Yes", "Shipped Date and Time" : date, "Estimated Time of Delivery": etd} 
        URL = "https://script.google.com/macros/s/AKfycbyOnM2yLrFUOpUQArKmRHV6owhFU_T4KiUTPYDpshCg2fwdRs_lVy_S/exec"
        response = requests.get(URL, params=parameters)
        print(response.content)
        
    def update2(self, order_id, city, item):   
        now = datetime.datetime.now()
        dt_string = now.strftime("%d/%m/%Y %H:%M:%S")
        NextDay_Date = datetime.datetime.today() + datetime.timedelta(days=1)
        etd = NextDay_Date.strftime("%d/%m/%Y %H:%M:%S")
        if item == 'Medicine':
          priority='HP'
          cost = '450'
        elif item == 'Food':
          priority='MP'
          cost = '250'
        elif item == 'Clothes':
          priority='LP'
          cost = '150'
        parameters = {"id":"OrdersShipped","Team Id": "VB#0323","Order ID": order_id,"City" :city, "Item": item, "Priority":priority, "Shipped Quantity" : '1',"Cost" : cost, "Shipped Status": "Yes", "Shipped Date and Time" : dt_string, "Estimated Time of Delivery": etd} 
        URL = "https://script.google.com/macros/s/AKfycbw5xylppoda-8HPjt2Tzq4ShU_Xef-Ik-hEtBPcPk0gdGw8095j4RZ7/exec"
        response = requests.get(URL, params=parameters)
        print(response.content)


    def sorting_bins(self, color, index):                                                        
        ur5=Camera1('ur5_2')

        if color=="yellow":
            rospy.sleep(0.33)
            ur5.Conveyer(0)
            rospy.sleep(1)
	    ur5.enableGripper()
            ur5.Conveyer(100)
	    ur5.go_to_yellowbin_back(index)
               

        if color=="red":
            rospy.sleep(0.31)
            ur5.Conveyer(0)
            rospy.sleep(1)
	    ur5.enableGripper()
            ur5.Conveyer(100)
	    ur5.go_to_redbin_back(index)

            
        if color=="green":
            rospy.sleep(0.30)
            ur5.Conveyer(0)
            rospy.sleep(1)
	    ur5.enableGripper()
            ur5.Conveyer(100)
	    ur5.go_to_greenbin_back(index)
 
    def callback(self): 
        
        tf_obj=TF()
        self.Conveyer(100)    
        
        # 1. Go to Home Position
        rospy.logwarn("10. Playing start to home Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'start_to_home.yaml', 5)
        i=0
        while not rospy.is_shutdown():
            try:#subscribing to logical camera
                msg_1 = rospy.wait_for_message("/eyrc/vb/logical_camera_1",LogicalCameraImage)
                msg_2 = rospy.wait_for_message("/eyrc/vb/logical_camera_2",LogicalCameraImage)
                tf_obj.callback_image(msg_2)
                self.pkg=msg_2.models[-1].type
                rospy.logwarn(self.pkg)
                self.y_val=msg_2.models[-1].pose.position.y

                self.temp="{:.2f}".format(self.y_val)
                rospy.logwarn(self.temp)

                md_type = tf_obj.model.type #savin model name for reference
                rospy.logwarn(md_type) 
                #if package is 1 stop conveyor
            
                colour_type= od[md_type]
                #colour_type = od[str(md_type)]
                if not md_type == '' :
	            self.Conveyer(79)
                    self.sorting_bins(colour_type,i)
                    i=i+1
            except IndexError:
                rospy.logwarn("index not found")

        
            
def main():
    #rospy.sleep(60)
    rospy.init_node('node_t5_ur5_2', anonymous=True)
    ur5=Camera1('ur5_2')
    t1 = threading.Thread(target = ur5.update_list)
    t1.start()
    ur5.callback()
   
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
