#!/usr/bin/env python

import rospy
from pkg_task4.msg import task5
from PIL import Image, ImageDraw
import random
import rospy
import cv2
import sys
import time
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from collections import OrderedDict 
import numpy as np
import pyzbar.pyzbar as pyzbar
import requests

class Camera1:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,self.callback)

    # 1. Create a handle to publish messages to a topic.
    self.var_handle_pub = rospy.Publisher('my_topic', task5, queue_size=10)
    self.var_loop_rate = rospy.Rate(1)

  def get_qr_data(self, arg_image):
    qr_result = pyzbar.decode(arg_image)
    #print qr_result

    #print len(qr_result)

    #rospy.loginfo(qr_result[0].data)
    list_x = []
    list_y = []
    list_a = []
    color=[]
    val_dict={}
    print len( qr_result)
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
  #------------spreadsheet updating function-------------------------------------------------------
  def update(self,box_color, storage_number, num):   
    if box_color == 'red':
      priority='HP'
      cost=450
      item='Medicine'
    elif box_color == 'yellow':
      priority='MP'
      cost=250
      item='Food'
    elif box_color == 'green':
      priority='LP'
      cost=150
      item='Clothes'
    localtime = time.localtime(time.time())
    month = '0' + str(localtime.tm_mon)
    year = str(localtime.tm_year)
    SKU = box_color[0].upper()+str(num)+month+year[2]+year[3]
    parameters = {"id":"Inventory", "Team Id": "VB#0323", "Unique Id":"QwertyTS", "SKU" :SKU,"Item": item, "Priority":priority, "Storage Number":storage_number,"Cost" : cost, "Quantity" : '1'} 
    URL = "https://script.google.com/macros/s/AKfycbyOnM2yLrFUOpUQArKmRHV6owhFU_T4KiUTPYDpshCg2fwdRs_lVy_S/exec"
    response = requests.get(URL, params=parameters)
    print(response.content)
  
  def update_2(self,box_color, storage_number, num):   
    if box_color == 'red':
      priority='HP'
      cost=450
      item='Medicine'
    elif box_color == 'yellow':
      priority='MP'
      cost=250
      item='Food'
    elif box_color == 'green':
      priority='LP'
      cost=150
      item='Clothes'
    localtime = time.localtime(time.time())
    month = '0' + str(localtime.tm_mon)
    year = str(localtime.tm_year)
    SKU = box_color[0].upper()+str(num)+month+year[2]+year[3]
    parameters = {"id":"Inventory", "Team Id": "VB#0323", "SKU" :SKU,"Item": item, "Priority":priority, "Storage Number":storage_number,"Cost" : cost, "Quantity" : '1'} 
    URL = "https://script.google.com/macros/s/AKfycbw5xylppoda-8HPjt2Tzq4ShU_Xef-Ik-hEtBPcPk0gdGw8095j4RZ7/exec"
    response = requests.get(URL, params=parameters)
    print(response.content)
  
   
    

    
    '''callback function to read the image, denoise it and then update the dictionary od with storage number and color''' 
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)

    (rows,cols,channels) = cv_image.shape
    
    image = cv_image
    font = cv2.FONT_HERSHEY_PLAIN
    # Resize a 720x1280 image to 360x640 to fit it on the screen
    resized_image_1 = cv2.resize(image, (720, 1280)) 
    grayImage = cv2.cvtColor(resized_image_1, cv2.COLOR_BGR2GRAY) 
    (thresh, blackAndWhiteImage) = cv2.threshold(grayImage, 10, 255, cv2.THRESH_BINARY)
    resized_image_2 = cv2.resize(image, (720+543, 1280+543)) 
    image3 = blackAndWhiteImage
    list_b=[]
    #list_b= self.get_qr_data(blackAndWhiteImage)
    list_b= self.get_qr_data(resized_image_1)
    detectedBarcodes = pyzbar.decode(image3)
 
    for barcode in detectedBarcodes:
     
        (x, y, w, h) = barcode.rect
        cv2.rectangle(image3, (x, y), (x + w, y + h), (255, 0, 255), 1)
        barcodeData = barcode.data.decode("utf-8")
	# draw the barcode data and barcode type on the image
	text = "{}".format(barcodeData)
	cv2.putText(image3, text, (x, y - 10),
		font, 2, (0, 0, 210), 1)
    resized_image_3 = cv2.resize(resized_image_2, (720/2, 1280/2))
    cv2.imshow("/eyrc/vb/camera_1/image_raw", blackAndWhiteImage)
    cv2.imshow("/eyrc/vb/camera_1/image_raw", resized_image_1)
    cv2.imshow("/eyrc/vb/camera_1/image_raw", grayImage)
    #cv2.imshow("/eyrc/vb/camera_1/image_raw", image3)
    cv2.waitKey(0)


def main(args):
  #rospy.sleep(20)
  rospy.init_node('node_t5_qr_decode', anonymous=True)

  ic = Camera1()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    rospy.loginfo("Shutting down")
  
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

