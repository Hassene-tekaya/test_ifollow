#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
A small example subscriber
"""
import rospy
from geometry_msgs.msg import Twist
import paho.mqtt.client as paho



def on_message(mosq, obj, msg):
    global command
    command = str(msg.payload.decode("utf-8"))
    print(command)
    mosq.publish('pong', 'ack', 0)
    print (command)
    if command=='up':
     move.linear.x += 0.01  # 
     print("move.linear.x = ", move.linear.x)
     pub.publish(move)
    elif command=='down':
     move.linear.x -= 0.01  # 
     print("move.linear.x = ", move.linear.x)
     pub.publish(move)
    elif command=='right':
     move.angular.z += 0.1  # 
     print("move.linear.z = ", move.linear.z)
     pub.publish(move)
    elif command=='left':
     move.angular.z -= 0.1  #
     print("move.linear.z = ", move.linear.z)
     pub.publish(move) 

def on_publish(mosq, obj, mid):
    pass

if __name__ == '__main__':
        rospy.init_node('topic_publisher')
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
#while not rospy.is_shutdown(): 
  #rate.sleep()

