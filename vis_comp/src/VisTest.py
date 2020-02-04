# -*- coding: utf-8 -*-
"""
Created on Tue Mar  7 16:10:38 2017

@author: computing
"""

import rospy
import cv2
import numpy as np
import math
import nav_msgs
import rosgraph_msgs
import pdb
from cv2 import namedWindow, cvtColor, imshow
from cv2 import destroyAllWindows, startWindowThread
from cv2 import COLOR_BGR2GRAY
from cv2 import blur, Canny
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from math import atan2
from math import degrees
from skimage.measure import compare_ssim
from skimage.transform import warp_polar

class VisualCompass():

  def __init__(self):
    # Determines how often images are taken during learning phase
    self.snap_interval = 5
    self.list_length = 0
    self.prev_list_length = 0
    # vector which stores visual snapshots (electric sheep free)
    self.stored_memories = []
    self.stored_vector = []
    self.cur_vector = Twist()
    self.min_ctr_val = 0
    #Thresholds
    self.follow_flag = False
    self.memory_match = False
    self.lower_thresh = 0.70
    self.upper_thresh = 1.00
    self.ctr = 0
    self.search_ctr = 1
    self.cb_ctr = 0
    self.var = 1
    #similarity metric
    self.cur_score = 0
    self.min_score = -1
    #vectors used for search step
    self.stop_vector = Twist()
    self.stop_vector.angular.x = 0.0
    self.stop_vector.angular.y = 0.0
    self.stop_vector.angular.z = 0.0
    self.stop_vector.linear.x = 0.0
    self.stop_vector.linear.y = 0.0
    self.stop_vector.linear.z = 0.0

    #rotate at 30 degrees per second
    self.spin_vector = Twist()
    self.spin_vector.angular.x = 0.0
    self.spin_vector.angular.y = 0.0
    self.spin_vector.angular.z = 1.04
    self.spin_vector.linear.x = 0.0
    self.spin_vector.linear.y = 0.0
    self.spin_vector.linear.z = 0.0

    self.forward_vector = Twist()
    self.forward_vector.angular.x = 0.0
    self.forward_vector.angular.y = 0.0
    self.forward_vector.angular.z = 0.0
    self.forward_vector.linear.x = 3.0
    self.forward_vector.linear.y = 0.0
    self.forward_vector.linear.z = 0.0
    #name of the robot for use in publishing to topics
    #(perhaps load from ROS parameters in future?)
    self.robot_name = "omni"
    self.img_ob =Image()
    #create window for display of camera topic
    cv2.namedWindow("Camera View", 1)
    cv2.namedWindow("Unfurled View", 1)
    cv2.startWindowThread()
    self.bridge = CvBridge()
    startWindowThread()

    #define publisher and subscriber for the robot motion and camera feed respectively
    self.motion_pub = rospy.Publisher(self.robot_name+
    "/wheel_base_controller/cmd_vel",Twist, queue_size=10)
    self.image_sub = rospy.Subscriber(self.robot_name+
    "/camera1/image_raw",Image,self.callback, queue_size=10)
    #self.motion_sub = rospy.Subscriber(self.robot_name+
    #"/wheel_base_controller/cmd_vel",Twist,self.callback queue_size=10)

  def callback(self,some_img):
    cam_view = self.get_img(some_img)
    

  def get_img(self, some_img):
    try:
      #convert image data from topic into open cv format
      input_img = self.bridge.imgmsg_to_cv2(some_img, "bgr8")
    except CvBridgeError, e:
      print e
    #convert to greyscale for simpler image processing (perhaps use colour images in future)
    grey_img = cvtColor(input_img, COLOR_BGR2GRAY)
    
    #im2 = self.unfurl_img(grey_img)
    #cv2.imshow("Unfurled View",im2)
    grey_img = warp_polar (grey_img,(160,160), radius = 160,output_shape= (160,(160*math.pi)))
    cv2.imshow("Camera View", grey_img)
    return grey_img
    

if __name__ == "__main__":
  cv2.startWindowThread()
  rospy.init_node('ant_VC_node', anonymous=True)
  avc = VisualCompass()
  rospy.spin()
