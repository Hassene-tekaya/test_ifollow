#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
import std_srvs.srv
from typing import NamedTuple
from dt_apriltags import Detector
import os
import time
import cv2
from cv2 import imshow
import numpy as np
import quaternion

class Artag (NamedTuple):
   id_number: int
   pos_x : float
   pos_y : float
   
ar_tag_1 = Artag(20, -0.55, -1.65)
ar_tag_2 = Artag(21, -0.55, -0.15)
ar_tag_3 = Artag(22,  1.85,  0.05)

GOAL_POSE = [ar_tag_1, ar_tag_2, ar_tag_3]

test_images_path = 'test_files'

visualization = False

at_detector = Detector(families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

        
if __name__ == '__main__':

#### DETECT APRIL TAG FROM IMAGE ####

 print("\n\nTESTING WITH A SAMPLE IMAGE")

 img = cv2.imread(test_images_path+'/'+ 'ar_tag_1.JPG', cv2.IMREAD_GRAYSCALE)
 cameraMatrix = np.array([336.7755634193813, 0.0, 333.3575643300718, 0.0, 336.02729840829176, 212.77376312080065, 0.0, 0.0, 1.0]).reshape((3,3))
 camera_params = ( cameraMatrix[0,0], cameraMatrix[1,1], cameraMatrix[0,2], cameraMatrix[1,2] )

 if visualization:
    cv2.imshow('Original image',img)

 tags = at_detector.detect(img, True, camera_params, 0.065)
 #print(tags)
 for tag in tags:
  print ('tag_id = ', str(tag.tag_id))
  #print (str(tag.pose_R))
  tag_detected_id = tag.tag_id
  orient_nav_goal = np.quaternion(1,0,0,0)
  print(orient_nav_goal)
 
 
 #### MATCHING WITH ID POSTION AND NAVIGATE TO GOAL ####
 rospy.init_node('nav2goal')
 pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
 goalmsg = PoseStamped()
 rate = rospy.Rate(1)




#### MATCHING WITH ID POSTION ####
 i = 0
 while i < len(GOAL_POSE):
  if GOAL_POSE[i].id_number == tag_detected_id:
    goalmsg.pose.position.x = GOAL_POSE[i].pos_x
    goalmsg.pose.position.y = GOAL_POSE[i].pos_y
    goalmsg.pose.orientation = orient_nav_goal
    break
  i += 1


#### NAVIGATE TO GOAL ####   
 goalmsg.header.frame_id = "map"
 goalmsg.header.stamp = rospy.Time.now()
 rospy.loginfo("going to pose id: "+str (tag_detected_id))
 rospy.loginfo("going to pose \n"+str (goalmsg.pose))
 while not rospy.is_shutdown():
            #rospy.sleep(2)
            clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', std_srvs.srv.Empty)
            rospy.sleep(1)
            pub.publish(goalmsg)
   
