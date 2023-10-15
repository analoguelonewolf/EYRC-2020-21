#! /usr/bin/env python

import rospy
import sys
import os
import copy
import math

from pkg_vb_sim.msg import Model 
from pkg_vb_sim.msg import LogicalCameraImage 
from pkg_vb_sim.srv import vacuumGripper
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib



class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('node_task3_soln', anonymous=True)

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

        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)  
        wpose.position.y = waypoints[0].position.y + (trans_y)  
        wpose.position.z = waypoints[0].position.z + (trans_z)
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

        # The reason for deleting the first two waypoints from the computed Cartisian Path can be found here,
        # https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/?answer=257488#post-id-257488
        num_pts = len(plan.joint_trajectory.points)
        if (num_pts >= 3):
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)


    #move Ur5 arm to predefined pose
    def go_to_predefined_pose(self, arg_pose_name):
        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')


    #move Ur5 arm to userdefined pose
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

    #rosservice function to enable gripper
    def enableGripper(self):
        rospy.wait_for_service('/eyrc/vb/ur5_1/activate_vacuum_gripper')
        try:
            activate=rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper',vacuumGripper)
	    activate(True)
	    rospy.loginfo('\033[94m' + ">>>vacuum gripper ready" + '\033[0m')

        except rospy.ServiceException:
            print('Service call failed')

    #rosservice function to diable gripper
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
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


class logical_camera_2:

    def callback_image(self,myMsg):
            try:
                models = []
                models.append(myMsg.models.pop())
            
            except Exception as e:
                rospy.loginfo("model not received!!!! ",e.__class__)                    

            else:
                self.model = models.pop()   #model_type, pose                
                

    

def main():
    ur5 = Ur5Moveit()
    lg = logical_camera_2()

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
    os.system('rosservice call /eyrc/vb/conveyor/set_power "power: 28"')        
        
    # 1. Go to Home Position
    ur5.set_joint_angles(lst_joint_angles_home)

    while(True):
       #wait for any of the package to arrive under logical_camera
        msg = rospy.wait_for_message("/eyrc/vb/logical_camera_2",LogicalCameraImage)
        lg.callback_image(msg)
        md_type = lg.model.type
        
        if(md_type == "packagen1"):
            rospy.sleep(1.8)
            # turn off conveyor 
            os.system('rosservice call /eyrc/vb/conveyor/set_power "power: 0"')
            #translate to the package
            ur5.ee_cartesian_translation(0,0,-0.015)                            
            #activate Vacuum gripper
            ur5.enableGripper()
            #place arm over red bin
            reached_red = ur5.set_joint_angles(lst_joint_angles_red)
            #deactivate vacuum gripper                                
            ur5.disableGripper()
            # turn on conveyor 
            os.system('rosservice call /eyrc/vb/conveyor/set_power "power: 15"')
            #place arm back to home pose
            ur5.set_joint_angles(lst_joint_angles_home)
            
        elif(md_type == "packagen2"):
            rospy.sleep(3.1)
            # turn off conveyor 
            os.system('rosservice call /eyrc/vb/conveyor/set_power "power: 0"')
            rospy.sleep(1)
            #translate to the package
            ur5.ee_cartesian_translation(0.18,0.015,-0.015)
            #activate Vacuum gripper
            ur5.enableGripper()
            #place arm over green bin
            reached_green = ur5.set_joint_angles(lst_joint_angles_green)
            #deactivate vacuum gripper                                
            ur5.disableGripper()
            # turn on conveyor 
            os.system('rosservice call /eyrc/vb/conveyor/set_power "power: 17"')
            #place arm back to home pose
            ur5.set_joint_angles(lst_joint_angles_home)
            
        elif(md_type == "packagen3"):
            rospy.sleep(0.9)
            # turn off conveyor 
            os.system('rosservice call /eyrc/vb/conveyor/set_power "power: 0"')
            rospy.sleep(1)
            #translate to the package
            ur5.ee_cartesian_translation(-0.085,0.085,-0.015)
            #activate Vacuum gripper
            ur5.enableGripper()
            #place arm over blue Bin
            reached_blue = ur5.set_joint_angles(lst_joint_angles_blue)
            # turn on conveyor 
            os.system('rosservice call /eyrc/vb/conveyor/set_power "power: 11"')
            #deactivate vacuum gripper                                
            ur5.disableGripper()
            #place arm back to home pose
            ur5.set_joint_angles(lst_joint_angles_home)
            break

    rospy.spin()        

    rospy.loginfo('\033[94m' + "Task Completed Succesfully" + '\033[0m')

    del lg        
    del ur5


if __name__ == '__main__':
    main()

