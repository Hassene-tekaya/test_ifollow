#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
from service_node.srv import *
import rospy


def handle_trig(req):
   print('Triggering webcam to capture image')

if __name__ == '__main__':
   rospy.init_node('service_node')
   rospy.Service('Trigger', trigger_srv.srv.empty, handle_trig)
   rospy.spin()
