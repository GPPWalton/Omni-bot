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
from cv2 import namedWindow, cvtColor, imshow,threshold,imwrite
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
#these are deprecated, replace soon.
from skimage.measure import compare_ssim,compare_nrmse,compare_psnr

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
    self.lower_thresh = 0.999
    self.upper_thresh = 1.00
    self.ctr = 0
    self.search_ctr = 1
    self.cb_ctr = 0
    self.var = 1
    #similarity metric
    self.cur_score = 0
    self.max_score = -1
    self.cur_mse= 0
    self.min_mse = 100
    self.cur_psnr = 0
    self.max_psnr = -1
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
    self.spin_vector.angular.z = 1.00
    self.spin_vector.linear.x = 0.0
    self.spin_vector.linear.y = 0.0
    self.spin_vector.linear.z = 0.0

    self.forward_vector = Twist()
    self.forward_vector.angular.x = 0.0
    self.forward_vector.angular.y = 0.0
    self.forward_vector.angular.z = 0.0
    self.forward_vector.linear.x = -3.0
    self.forward_vector.linear.y = 0.0
    self.forward_vector.linear.z = 0.0
    #name of the robot for use in publishing to topics
    #(perhaps load from ROS parameters in future?)
    self.robot_name = "omni"
    self.rate = rospy.Rate(1)
    #create window for display of camera topic
    cv2.namedWindow("Camera View", 1)
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
    if(self.list_length < self.prev_list_length):
      self.memory_match = False
    #on enter key-press, take photo
    if(self.var==1):
      if(self.ctr < 5):
        key_press = raw_input("Press enter to take snapshot: ")
        if (key_press ==""):
          if(self.ctr == 4):
            self.list_length = self.ctr
            self.var+=1
            #pdb.set_trace()
          self.stored_memories.append(cam_view)
          print(len(self.stored_memories))
          
          self.ctr +=1
    elif("2" in str(self.var)):
      if(self.var == 2):
        self.var += .25
      if(self.var == 2.25):
        if(self.search_ctr <= 16):
          #rotate robot until the closest match is found, storing the value that is closest to 0 (a perfect match)
          self.cur_score,self.cur_mse, self.cur_psnr = self.compare_memories(self.stored_memories[len(self.stored_memories)-1],cam_view)
          #check if difference between current score and perfect score is less
          #than the previously found minimum
          if(self.cur_score > self.max_score):
            if(self.cur_mse < self.min_mse):
              if(self.cur_psnr > self. max_psnr):
                self.max_score = self.cur_score
                self.min_mse = self.cur_mse
                self.max_psnr = self.cur_psnr
                self.min_ctr_val = self.search_ctr
              print("CLOSEST MATCH^")
            #pdb.set_trace()
            #if((self.cur_score >= self.lower_thresh) & (self.cur_score <= self.upper_thresh)):
              #self.search_ctr = 16
          #else:
          self.motion_pub.publish(self.spin_vector)
          #sleep for a second
          rospy.sleep(1.0)
          self.motion_pub.publish(self.stop_vector)
          self.search_ctr +=1
        else:
          rotate_ctr = 0
          while(rotate_ctr < self.min_ctr_val):
            self.motion_pub.publish(self.spin_vector)
            #sleep for a second
            rospy.sleep(1.0)
            rotate_ctr +=1
          self.motion_pub.publish(self.stop_vector)
          self.var += .25
          #pdb.set_trace()
      elif(self.var == 2.50):
      #Travel for set amount of time until at approximate point
      ###IMPORTANT: This section was added into a loop because the machine this code
      #was being developed on was having issues with time keeping.
        run_ctr = 0;
        time_begin = rospy.Time.now()
        while(run_ctr < 10):
          self.motion_pub.publish(self.forward_vector)
          rospy.sleep(rospy.Duration(1.0))
          run_ctr += 1
        time_end = rospy.Time.now()
        duration = time_end - time_begin
        print("Slept for " + str(duration.to_sec()) + " secs")
        
        
        #stop robot
        self.motion_pub.publish(self.stop_vector)
        #flip memory_match flag
        self.memory_match = True
        #store length of stored memory list for if statement
        self.prev_list_length = self.ctr
        self.list_length = len(self.stored_memories)
        self.var += .25
        #pdb.set_trace()
      elif(self.var == 2.75):
        #reset variables used
        #if last memory, don't delete
        if(not(len(self.stored_memories) == 1)):
          self.stored_memories.pop()
        self.reinit_vars()
        #pdb.set_trace()
        
  def reinit_vars(self):
    #redefine variables that are used in algorithm to initial state
    self.min_ctr_val = 0
    #Thresholds
    self.follow_flag = False
    self.memory_match = False
    self.lower_thresh = 0.999
    self.upper_thresh = 1.00
    self.ctr = 0
    self.search_ctr = 1
    self.var = 2
    #similarity metric
    self.cur_score = 0
    self.max_score = -1
    self.cur_mse= 0
    self.min_mse = 100
    self.cur_psnr = 0
    self.max_psnr = -1

  def get_img(self, some_img):
    try:
      #convert image data from topic into open cv format
      input_img = self.bridge.imgmsg_to_cv2(some_img, "bgr8")
    except CvBridgeError, e:
      print e
    #convert to greyscale for simpler image processing (perhaps use colour images in future)
    grey_img = cvtColor(input_img, COLOR_BGR2GRAY)
    #ret,bin_img = threshold(grey_img,127,255,cv2.THRESH_BINARY)
    cv2.imshow("Camera View", grey_img)
    return grey_img

  def compare_memories(self,stored_memory,cur_memory):
    #mse = np.sum((stored_memory.astype("float") - cur_memory.astype("float")) ** 2)
    #mse /= float(stored_memory.shape[0] * stored_memory.shape[1])
    mse = compare_nrmse(stored_memory,cur_memory,norm_type="euclidean")
    psnr = compare_psnr(stored_memory,cur_memory)
    (score,diff) = compare_ssim(stored_memory,cur_memory,full=True)
    print(str(score) + " " + str(mse) + " " + str(psnr))
    return score,mse, psnr


if __name__ == "__main__":
  cv2.startWindowThread()
  rospy.init_node('ant_VC_node', anonymous=True)
  avc = VisualCompass()
  rospy.spin()
