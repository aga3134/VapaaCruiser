#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import math

class JoyControl():
    def __init__(self):
        self.pub = rospy.Publisher("car_cmd", Twist, queue_size=1)
        self.sub = rospy.Subscriber("joy", Joy, self.ReceiveJoyCmd)
        self.updateRate = rospy.get_param("~updateRate",30)
        self.turn = 0.0
        self.forward = 0.0

    def ReceiveJoyCmd(self,cmd):
        self.turn = -cmd.axes[0]
        speed = math.sqrt(cmd.axes[0]*cmd.axes[0]+cmd.axes[1]*cmd.axes[1])
        #use speed with sign of axes[1]
        self.forward = math.copysign(speed, cmd.axes[1])

    def PublishCommand(self):
        msg = Twist()
        msg.linear.x = self.forward
        msg.angular.z = self.turn
        self.pub.publish(msg)

    def Run(self):
        rate = rospy.Rate(self.updateRate)
        while not rospy.is_shutdown():
            self.PublishCommand()
            rate.sleep() 


if __name__ == '__main__':
    rospy.init_node('joy_mapper_node')
    rospy.loginfo("joy_mapper_node started")
    jc = JoyControl()
    jc.Run()