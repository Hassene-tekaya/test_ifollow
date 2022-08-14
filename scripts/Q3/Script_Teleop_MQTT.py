#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import paho.mqtt.publish as publish

import sys,tty,termios
class _Getch:
    def __call__(self):
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(3)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch

host = "localhost"

if __name__ == '__main__':
   while(1):
        inkey = _Getch()
        while(1):
                k=inkey()
                if k!='':break
        if k=='\x1b[A':
                print ("up")
                publish.single(topic="ROS/cmd_web", payload="up", hostname=host)
        elif k=='\x1b[B':
                print ("down")
                publish.single(topic="ROS/cmd_web", payload="down", hostname=host)
        elif k=='\x1b[C':
                print ("right")
                publish.single(topic="ROS/cmd_web", payload="left", hostname=host)
        elif k=='\x1b[D':
                print ("left")
                publish.single(topic="ROS/cmd_web", payload="right", hostname=host)
        else:
                print ("not an arrow key!")

