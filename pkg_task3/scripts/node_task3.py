#! /usr/bin/env python
#importing libraries
import rospy
import sys
import copy

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from pkg_vb_sim.msg import Model 
from pkg_vb_sim.msg import LogicalCameraImage 
from pkg_vb_sim.srv import ConveyorBeltControl
from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.srv import vacuumGripper
import tf2_ros
import tf2_msgs.msg
import math

#declaring global variables
p=0.0
q=0.0
r=0.0

class CartesianPath:

    # Constructor
    def __init__(self):

        rospy.init_node('node_task3.py', anonymous=True)

        self._planning_group = "ur5_1_planning_group"
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
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')


    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        # 1. Create a empty list to hold waypoints
        waypoints = []

        # 2. Add Current Pose to the list of waypoints
        waypoints.append(self._group.get_current_pose().pose)

        # 3. Create a New waypoint to follow world coordinates
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = trans_x  
        wpose.position.y = trans_y  
        wpose.position.z = trans_z
        # This to keep EE parallel to Ground Plane
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5


        # 4. Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))


        # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0)         # Jump Threshold
        rospy.loginfo("Path computed successfully. Moving the arm.")

        
        num_pts = len(plan.joint_trajectory.points)
        if (num_pts >= 3):
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)

    #make arm move to coordinates
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
            rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    
    # to set joint angles of ur5 to user user defined values
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



    #rosservice function to power conveyer belt
    def Conveyer(self,speed):
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        try:
            
            activate=rospy.ServiceProxy('/eyrc/vb/conveyor/set_power',conveyorBeltPowerMsg)
	    activate(speed)
	    rospy.loginfo('\033[94m' + ">>>conveyer belt not in motion" + '\033[0m')

        except rospy.ServiceException:
            print('Service call failed')

    #rosservice function to enable gripper
    def enableGripper(self):
        rospy.wait_for_service('/eyrc/vb/ur5_1/activate_vacuum_gripper')
        try:
            
            activate=rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper',vacuumGripper)
	    activate(True)
	    rospy.loginfo('\033[94m' + ">>>vacuum gripper ready" + '\033[0m')

        except rospy.ServiceException:
            print('Service call failed')
    
    #rosservice function to disable gripper
    def disableGripper(self):
        rospy.wait_for_service('/eyrc/vb/ur5_1/activate_vacuum_gripper')
        try:
            
            activate=rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper',vacuumGripper)
	    activate(False)
	    rospy.loginfo('\033[94m' + ">>>vacuum gripper ready" + '\033[0m')

        except rospy.ServiceException:
            print('Service call failed')

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')

#class to save coordinates from tf logical camera
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


def main():
    ur5 = CartesianPath()
    tf_obj=TF()
    # 1. Go to home position
    box_length = 0.15               # Length of the Package
    vacuum_gripper_width = 0.115    # Vacuum Gripper Width
    delta = vacuum_gripper_width + (box_length/2)  # 0.19
    
    

    #joint angles to bring ur5 arm over conveyor belt
    lst_joint_angles_home = [math.radians(7),
                          math.radians(-140),
                          math.radians(-57),
                          math.radians(-73),
                          math.radians(92),
                          math.radians(-83)]


    #joint angles to bring ur5 arm over Red Container(where red package is to be placed)
    lst_joint_angles_red = [math.radians(-79),
                          math.radians(-112),
                          math.radians(-94),
                          math.radians(-71),
                          math.radians(92),
                          math.radians(-169)]

    #joint angles to bring ur5 arm over green Container(where green package is to be placed)
    lst_joint_angles_green = [math.radians(-9),
                          math.radians(-41),
                          math.radians(87),
                          math.radians(-135),
                          math.radians(-89),
                          math.radians(81)]

    #joint angles to bring ur5 arm over blue Container(where blue package is to be placed)
    lst_joint_angles_blue = [math.radians(97),
                          math.radians(-105),
                          math.radians(-113),
                          math.radians(-51),
                          math.radians(92),
                          math.radians(7)]




    # turn on conveyor 
    ur5.Conveyer(45)    
        
    # 1. Go to Home Position
    ur5.set_joint_angles(lst_joint_angles_home)
 
    
    #defining reference frames for logical camera
    reference_frame = "world"
    target_frame_1 =  "logical_camera_2_packagen1_frame"
    target_frame_2 =  "logical_camera_2_packagen2_frame"
    target_frame_3 =  "logical_camera_2_packagen3_frame"

    while not rospy.is_shutdown():
        #subscribing to logical camera
        msg = rospy.wait_for_message("/eyrc/vb/logical_camera_2",LogicalCameraImage)
        tf_obj.callback_image(msg)
        md_type = tf_obj.model.type #savin model name for reference 
        #if package is 1 stop conveyot
        if(md_type == "packagen1"):
            ur5.Conveyer(0)
            p,q,r = tf_obj.func_tf_print(reference_frame,target_frame_1) #giving reference frames to function callback
            ur5.ee_cartesian_translation(float(p),float(q),float(r)+delta)  #putting values in cartesian fucntion
            ur5.enableGripper()
            reached_red = ur5.set_joint_angles(lst_joint_angles_red) #going to red bin
            if(not reached_red):
                rospy.loginfo("planning to red container Failed!!! \n")
                break
            ur5.disableGripper()
            ur5.Conveyer(39) 
            ur5.set_joint_angles(lst_joint_angles_home) #coming back to starting position

        #repeating the same for package 2
        elif(md_type == "packagen2"):
            ur5.Conveyer(0)
            p,q,r= tf_obj.func_tf_print(reference_frame,target_frame_2)
	    print(float(p))
	    print(float(q))
	    print(float(r))
            ur5.ee_cartesian_translation(float(p),float(q),float(r)+delta)                           
            ur5.enableGripper()
            reached_green = ur5.set_joint_angles(lst_joint_angles_green)
            if(not reached_green):
                rospy.loginfo("planning to green container Failed!!! \n")
                break
            ur5.disableGripper()
            ur5.Conveyer(26) 
            ur5.set_joint_angles(lst_joint_angles_home)

        #repeating the same for package 3
        elif(md_type == "packagen3"):
            ur5.Conveyer(0)
            p,q,r= tf_obj.func_tf_print(reference_frame,target_frame_3)
            ur5.ee_cartesian_translation(float(p),float(q),float(r)+delta)                            
            ur5.enableGripper()
            reached_blue = ur5.set_joint_angles(lst_joint_angles_blue)
            if(not reached_blue):
                rospy.loginfo("planning to blue container Failed!!! \n")
                break          
            ur5.disableGripper()
            ur5.set_joint_angles(lst_joint_angles_home)

    rospy.spin()       
    del ur5
    del tf_obj

if __name__ == '__main__':
    main()




    
