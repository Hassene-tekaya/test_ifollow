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

#### TRANSFORM ROTATION MATRIX TO QUATERNION ####                     
def RotationMatrixToQuaternion(m):
  t = np.matrix.trace(m)
  q_array = np.asarray([0.0, 0.0, 0.0, 0.0], dtype=np.float64)
  q = np.quaternion(1,0,0,0)

  if(t > 0):
    t = np.sqrt(t + 1)
    q_array[0] = 0.5 * t
    t = 0.5/t
    q_array[1] = (m[2,1] - m[1,2]) * t
    q_array[2] = (m[0,2] - m[2,0]) * t
    q_array[3] = (m[1,0] - m[0,1]) * t

  else:
    i = 0
    if (m[1,1] > m[0,0]):
        i = 1
    if (m[2,2] > m[i,i]):
        i = 2
    j = (i+1)%3
    k = (j+1)%3

    t = np.sqrt(m[i,i] - m[j,j] - m[k,k] + 1)
    q_array[i] = 0.5 * t
    t = 0.5 / t
    q_array[0] = (m[k,j] - m[j,k]) * t
    q_array[j] = (m[j,i] + m[i,j]) * t
    q_array[k] = (m[k,i] + m[i,k]) * t
    
  q.w = q_array[0]
  q.x = q_array[1]
  q.y = q_array[2]
  q.z = q_array[3]

  return q
####################################################
        
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
  orient_nav_goal = np.quaternion.inverse (RotationMatrixToQuaternion(tag.pose_R))
  print(orient_nav_goal)
 
 
 #### MATCHING WITH ID POSTION AND NAVIGATE TO GOAL ####
 rospy.init_node('nav2goal_plus_orientation')
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
   
