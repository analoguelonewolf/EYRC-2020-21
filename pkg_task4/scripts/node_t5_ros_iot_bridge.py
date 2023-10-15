#!/usr/bin/env python

# ROS Node - Action Server - IoT ROS Bridge
import time
import re
import rospy
import actionlib
import threading
import requests
from pkg_task4.msg import msgMqttSub           # Message Class for MQTT Subscription Messages

from pyiot import iot                                   # Custom Python Module to perfrom MQTT Tasks
import paho.mqtt.client as mqtt             #import the client1

a = []

'''This sheet is to update the second sheet, Incoming Orders with the orders given by MQTT. We receive the city, details like longitude and latitude, priority, item.'''
 
class Camera1:

  def __init__(self):
    self._config_mqtt_server_url = "broker.mqttdashboard.com"
    self._config_mqtt_server_port = 1883
    self._config_mqtt_sub_topic = "/eyrc/vb/QwertyTS/orders"
    self._config_mqtt_pub_topic = "/eyrc/vb/QwertyTS/orders"
    self._config_mqtt_qos = 0
    self._config_mqtt_sub_cb_ros_topic = "/ros_iot_bridge/mqtt/sub"
    self._handle_ros_pub = rospy.Publisher(self._config_mqtt_sub_cb_ros_topic, msgMqttSub, queue_size=10)

    self.ret = iot.mqtt_subscribe_thread_start(  self.mqtt_sub_callback, 
                                                    self._config_mqtt_server_url, 
                                                    self._config_mqtt_server_port, 
                                                    self._config_mqtt_sub_topic, 
                                                    self._config_mqtt_qos   )
    if(self.ret == 0):
        rospy.loginfo("MQTT Subscribe Thread Started")
    else:
        rospy.logerr("Failed to start MQTT Subscribe Thread")

    '''This is a callback function for MQTT Subscriptions'''
  def mqtt_sub_callback(self, client, userdata, message):
    i=1
    while self.ret == 0 :
        payload = str(message.payload.decode("utf-8"))
        res = re.findall(r'\w+', payload)
   
        otd=res[5]+'/'+res[4]+'/'+res[3]+' '+res[6]+':'+res[7]+':'+res[8]
        lond=res[12]+'.'+res[13]+' '+res[14]
        latd=res[20]+'.'+res[21]+' '+res[22]
        #update(res[1],res[10],res[18],lond,latd,otd)
        #update2(res[1],res[10],res[18],lond,latd,otd)
        msg_mqtt_sub = msgMqttSub()
        msg_mqtt_sub.timestamp = rospy.Time.now()
        msg_mqtt_sub.topic = message.topic
        msg_mqtt_sub.orderID= res[10]
        msg_mqtt_sub.orderDT= otd
        msg_mqtt_sub.city= res[1]
        msg_mqtt_sub.long= lond
        msg_mqtt_sub.lat= latd
        msg_mqtt_sub.item= res[18]
        msg_mqtt_sub.quantity= str(1)
        if res[18] == 'Medicine':
            priority='HP'
            cost = '450'
        elif res[18] == 'Food':
            priority='MP'
            cost = '250'
        elif res[18] == 'Clothes':
            priority='LP'
            cost = '150'
        msg_mqtt_sub.priority= priority
        msg_mqtt_sub.cost= cost
        rospy.loginfo(msg_mqtt_sub)
        self._handle_ros_pub.publish(msg_mqtt_sub)
        time.sleep(1) # wait
        i=i+1
        break

#------------spreadsheet updating function-------------------------------------------------------
def update(city, order_id, item, lon, lat, ordertime): 
    priority=''
    cost=''
    if item == 'Medicine':
      priority='HP'
      cost = '450'
    elif item == 'Food':
      priority='MP'
      cost = '250'
    elif item == 'Clothes':
      priority='LP'
      cost = '150'
    parameters = {"id":"IncomingOrders", "Team Id": "VB#0323", "Order ID": order_id , "Order Date and Time": ordertime , "Item": item, "Priority":priority, "Order Quantity":"1","City" :city ,"Longitude": lon , "Latitude": lat,"Cost": cost} 
    URL = "https://script.google.com/macros/s/AKfycbyOnM2yLrFUOpUQArKmRHV6owhFU_T4KiUTPYDpshCg2fwdRs_lVy_S/exec"
    
    response = requests.get(URL, params=parameters)
    print(response.content)
#------------spreadsheet updating function-------------------------------------------------------
def update2(city, order_id, item, lon, lat, ordertime): 
    priority=''
    cost=''
    if item == 'Medicine':
      priority='HP'
      cost = '450'
    elif item == 'Food':
      priority='MP'
      cost = '250'
    elif item == 'Clothes':
      priority='LP'
      cost = '150'
    parameters = {"id":"IncomingOrders", "Team Id": "VB#0323", "Order ID": order_id , "Order Date and Time": ordertime , "Item": item, "Priority":priority, "Order Quantity":"1","City" :city ,"Longitude": lon , "Latitude": lat,"Cost": cost} 
    URL = "https://script.google.com/macros/s/AKfycbw5xylppoda-8HPjt2Tzq4ShU_Xef-Ik-hEtBPcPk0gdGw8095j4RZ7/exec"
    response = requests.get(URL, params=parameters)
    print(response.content)


def main():
    #rospy.sleep(50)
    rospy.init_node('node_t5_ros_iot_bridge', anonymous=True)

    ic = Camera1()

    rospy.spin()

# Python Main
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
