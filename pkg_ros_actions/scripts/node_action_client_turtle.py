#!/usr/bin/env python

# ROS Node - Action Client - IoT ROS Bridge

import rospy
import actionlib
import time

from pkg_iot_ros_bridge.msg import msgIotRosAction      # Message Class that is used by ROS Actions internally
from pkg_iot_ros_bridge.msg import msgIotRosGoal        # Message Class that is used for Goal Messages
from pkg_iot_ros_bridge.msg import msgIotRosResult      # Message Class that is used for Result Messages
from pkg_ros_actions.msg import myActionMsgAction       # Message Class that is used by ROS Actions internally
from pkg_ros_actions.msg import myActionMsgGoal         # Message Class that is used for Goal messages
from pkg_ros_actions.msg import myActionMsgResult       # Message Class that is used for Result messages

class IotRosBridgeActionClient:

    # Constructor
    def __init__(self):

        # Initialize Action Client
        self._ac = actionlib.ActionClient('/action_iot_ros',
                                          msgIotRosAction)
        
        # Dictionary to Store all the goal handels
        self._goal_handles = {}

        # Store the MQTT Topic on which to Publish in a variable
        param_config_iot = rospy.get_param('config_iot')
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']
	
	

        # Wait for Action Server that will use the action - '/action_iot_ros' to start
        self._ac.wait_for_server()
        rospy.loginfo("Action server up, we can send goals.")

    
    # This function will be called when there is a change of state in the Action Client State Machine
    def on_transition(self, goal_handle):
        
        # from on_goal() to on_transition(). goal_handle generated by send_goal() is used here.
        
        result = msgIotRosResult()

        index = 0
        for i in self._goal_handles:
            if self._goal_handles[i] == goal_handle:
                index = i
                break

        rospy.loginfo("Transition Callback. Client Goal Handle #: " + str(index))
        rospy.loginfo("Comm. State: " + str(goal_handle.get_comm_state()) )
        rospy.loginfo("Goal Status: " + str(goal_handle.get_goal_status()) )
        
        # Comm State - Monitors the State Machine of the Client which is different from Server's
        # Comm State = 2 -> Active
        # Comm State = 3 -> Wating for Result
        # Comm State = 7 -> Done
        
        # if (Comm State == ACTIVE)
        if goal_handle.get_comm_state() == 2:
            rospy.loginfo(str(index) + ": Goal just went active.")
        
        # if (Comm State == DONE)
        if goal_handle.get_comm_state() == 7:
            rospy.loginfo(str(index) + ": Goal is DONE")
            rospy.loginfo(goal_handle.get_terminal_state())
            
            # get_result() gets the result produced by the Action Server
            result = goal_handle.get_result()
            rospy.loginfo(result.flag_success)

            if (result.flag_success == True):
                rospy.loginfo("Goal successfully completed. Client Goal Handle #: " + str(index))
            else:
                rospy.loginfo("Goal failed. Client Goal Handle #: " + str(index))


    # This function is used to send Goals to Action Server
    def send_goal(self, arg_protocol, arg_mode, arg_topic, arg_message):
        # Create a Goal Message object
        goal = msgIotRosGoal()

        goal.protocol = arg_protocol
        goal.mode = arg_mode
        goal.topic = arg_topic
        goal.message = arg_message

        rospy.loginfo("Send goal.")
        
        # self.on_transition - It is a function pointer to a function which will be called when 
        #                       there is a change of state in the Action Client State Machine
        goal_handle = self._ac.send_goal(goal,
                                         self.on_transition,
                                         None)

        return goal_handle

class SimpleActionClientTurtle:

    # Constructor
    def __init__(self):
        self._sac = actionlib.SimpleActionClient('/action_turtle',
                                                myActionMsgAction)
        self._sac.wait_for_server()
        rospy.loginfo("Action server is up, we can send new goals!")

    # Function to send Goals to Action Servers
    def send_goal(self, arg_dis, arg_angle):
        
        # Create Goal message for Simple Action Server
        goal = myActionMsgGoal(distance=arg_dis, angle=arg_angle)
        
        '''
            * done_cb is set to the function pointer of the function which should be called once 
                the Goal is processed by the Simple Action Server.

            * feedback_cb is set to the function pointer of the function which should be called while
                the goal is being processed by the Simple Action Server.
        ''' 
        self._sac.send_goal(goal, done_cb=self.done_callback,
                           feedback_cb=self.feedback_callback)
        
        rospy.loginfo("Goal has been sent.")

    # Function print result on Goal completion
    def done_callback(self, status, result):
        rospy.loginfo("Status is : " + str(status))
        rospy.loginfo("Result is : " + str(result))
        

    # Function to print feedback while Goal is being processed
    def feedback_callback(self, feedback):
        rospy.loginfo(feedback)
    

# Main
def main():


    rospy.init_node('node_iot_ros_bridge_action_client')
    action_client = IotRosBridgeActionClient()
    obj_server_result = myActionMsgResult()
    obj_server_result.final_x = xfinal
    obj_server_result.final_y = yfinal
    obj_server_result.final_theta = thetafinal
     # 2. Create a object for Simple Action Client.
    obj_client = SimpleActionClientTurtle()

    # 3. Send Goals to Draw a hexagon
    obj_client.send_goal(2, 0)
    rospy.sleep(5)
    goal_handle1 = action_client.send_goal("mqtt", "pub", action_client._config_mqtt_pub_topic, str(xfinal) + str(yfinal) + str(thetafinal))
    action_client._goal_handles['2'] = goal_handle1
    rospy.loginfo("Goal #2 Sent")

    obj_client.send_goal(2, 60)
    rospy.sleep(5)
    goal_handle1 = action_client.send_goal("mqtt", "pub", action_client._config_mqtt_pub_topic, str(xfinal) + str(yfinal) + str(thetafinal))
    action_client._goal_handles['2'] = goal_handle1
    rospy.loginfo("Goal #2 Sent")

    obj_client.send_goal(2, 60)
    rospy.sleep(5)
    goal_handle1 = action_client.send_goal("mqtt", "pub", action_client._config_mqtt_pub_topic, str(xfinal) + str(yfinal) + str(thetafinal))
    action_client._goal_handles['2'] = goal_handle1
    rospy.loginfo("Goal #2 Sent")

    obj_client.send_goal(2, 60)
    rospy.sleep(5)
    goal_handle1 = action_client.send_goal("mqtt", "pub", action_client._config_mqtt_pub_topic, str(self._xfinal) + str(self._yfinal) + str(self._thetafinal))
    action_client._goal_handles['2'] = goal_handle1
    rospy.loginfo("Goal #2 Sent")

    obj_client.send_goal(2, 60)
    rospy.sleep(5)
    goal_handle1 = action_client.send_goal("mqtt", "pub", action_client._config_mqtt_pub_topic, str(xfinal) + str(yfinal) + str(thetafinal))
    action_client._goal_handles['2'] = goal_handle1
    rospy.loginfo("Goal #2 Sent")

    obj_client.send_goal(2, 60)
    rospy.sleep(5)
    goal_handle1 = action_client.send_goal("mqtt", "pub", action_client._config_mqtt_pub_topic, str(xfinal) + str(yfinal) + str(thetafinal))
    action_client._goal_handles['2'] = goal_handle1
    rospy.loginfo("Goal #2 Sent")
    
    obj_client.send_goal(2, 60)
    goal_handle1 = action_client.send_goal("mqtt", "pub", action_client._config_mqtt_pub_topic, str(xfinal) + str(yfinal) + str(thetafinal))
    action_client._goal_handles['2'] = goal_handle1
    rospy.loginfo("Goal #2 Sent")
    


    #goal_handle1 = action_client.send_goal("mqtt", "pub", action_client._config_mqtt_pub_topic, "Hello from Action Client!")
    #action_client._goal_handles['1'] = goal_handle1
    #rospy.loginfo("Goal #1 Sent")

    #goal_handle1 = action_client.send_goal("mqtt", "pub", action_client._config_mqtt_pub_topic, str(obj_server_result))
    #action_client._goal_handles['2'] = goal_handle1
    #rospy.loginfo("Goal #2 Sent")

    #goal_handle2 = action_client.send_goal("mqtt", "sub", "eyrc/vb/mqtt/myTopic", "NA")
    #action_client._goal_handles['3'] = goal_handle2
    #rospy.loginfo("Goal #3 Sent")

    rospy.sleep(1.0)
    goal_handle1.cancel()

    rospy.spin()


if __name__ == '__main__':
    main()
