#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import paho.mqtt.client as paho
import time



def on_message(mosq, obj, msg):
    global command
    command = str(msg.payload.decode("utf-8"))
    print(command)
    mosq.publish('pong', 'ack', 0)
    print (command)
    if command=='up':
     move.linear.x += 0.05  # 
     move.angular.z = 0 
     print("move.linear.x = ", move.linear.x)
     pub.publish(move)
     time.sleep(0.05)
     move.linear.x = 0
     print("move.linear.x = ", move.linear.x)
     pub.publish(move)
    elif command=='down':
     move.linear.x -= 0.05  # 
     move.angular.z= 0 
     print("move.linear.x = ", move.linear.x)
     pub.publish(move)
     time.sleep(0.05)
     move.linear.x = 0
     print("move.linear.x = ", move.linear.x)
     pub.publish(move)
    elif command=='right':
     move.linear.x = 0
     move.angular.z += 0.1 
     print("move.linear.z = ", move.linear.z)
     pub.publish(move)
     time.sleep(0.05)
     move.angular.z= 0  # 
     print("move.linear.z = ", move.linear.z)
     pub.publish(move)
    elif command=='left':
     move.linear.x = 0
     move.angular.z -= 0.1  #
     print("move.linear.z = ", move.linear.z)
     pub.publish(move) 
     time.sleep(0.05)
     move.angular.z= 0  # 
     print("move.linear.z = ", move.linear.z)
     pub.publish(move) 

def on_publish(mosq, obj, mid):
    pass

if __name__ == '__main__':
        rospy.init_node('MQTT_Listener')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(2)
move = Twist() # defining the way we can allocate the values
client = paho.Client()
client.on_message= on_message
client.on_publish = on_publish
client.connect("localhost", 1883, 60)
client.subscribe("ROS/cmd_web", 0)
command = client.on_message
while client.loop() == 0:
        pass


