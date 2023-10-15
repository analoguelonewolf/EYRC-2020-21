#!/usr/bin/env python

import rospy
from pkg_task4_1.msg import task4

import random
import rospy
import cv2
import sys
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from collections import OrderedDict 

from pyzbar.pyzbar import decode

class Camera1:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image, self.callback)
    # 1. Create a handle to publish messages to a topic.
    self.var_handle_pub = rospy.Publisher('my_topic', task4, queue_size=10)
    self.var_loop_rate = rospy.Rate(1)
    # 2. Initializes the ROS node for the process.


  
  def get_qr_data(self, arg_image):
    qr_result = decode(arg_image)
    #print qr_result

    #print len(qr_result)

    #rospy.loginfo(qr_result[0].data)
    list_x = []
    list_y = []
    list_a = []
    color=[]
    val_dict={}

    if ( len( qr_result ) > 0):
      for i in qr_result:
        key=math.ceil(i.polygon[0][0]/100)
        val_dict.setdefault(key,[])
        val_dict[key].append((math.ceil(i.polygon[0][1]/100),i.data))
        #val_dict[key].append(i.data)
        #val_dict[key].append(math.ceil(i.polygon[0][1]/100))
      for i in sorted (val_dict.keys()) :
        color.append(val_dict[i])
      list_a = sorted (color[0]) + sorted (color[1]) + sorted (color[2])
      
      return (list_a)
    else:
      return ('NA')

  def colour(self):
    global packagen00
    print packagen00



  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)

    (rows,cols,channels) = cv_image.shape
    
    image = cv_image

    # Resize a 720x1280 image to 360x640 to fit it on the screen
    resized_image = cv2.resize(image, (720/2, 1280/2))  
    dst = cv2.fastNlMeansDenoisingColored(resized_image, None, 10, 10, 7, 30) 
    cv2.imshow("/eyrc/vb/camera_1/image_raw", resized_image)
    
    list_b=[]
    list_b= self.get_qr_data(image)

    
    obj_msg = task4()

    obj_msg.packagen00 = str(list_b[0][1])
    obj_msg.packagen10 = str(list_b[1][1])
    obj_msg.packagen20 = str(list_b[2][1])
    obj_msg.packagen30 = str(list_b[3][1])
    #obj_msg.packagen01 = str(list_b[0][1])
    obj_msg.packagen11 = str(list_b[4][1])
    obj_msg.packagen21 = str(list_b[5][1])
    obj_msg.packagen31 = str(list_b[6][1])
    obj_msg.packagen02 = str(list_b[7][1])
    obj_msg.packagen12 = str(list_b[8][1])
    obj_msg.packagen22 = str(list_b[9][1])
    obj_msg.packagen32 = str(list_b[10][1])
    while not rospy.is_shutdown():
      rospy.loginfo(obj_msg)
      self.var_handle_pub.publish(obj_msg)
      #rospy.spin()
      rospy.sleep(1)

 
    cv2.waitKey(3)
   
  def ret():
    return od

def main():

    rospy.init_node('node_myMsg_talker', anonymous=True)

    ic = Camera1()

    rospy.spin()

# Python Main
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

