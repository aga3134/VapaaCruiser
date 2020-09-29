#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import serial
import threading
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

class SerialCommand():
    def __init__(self):
        self.port = rospy.get_param("~port","/dev/ttyUSB0")
        self.baud = rospy.get_param("~rightRatio",115200)

        rospy.loginfo("use port=%s baud=%d" % (self.port,self.baud))

        self.state = "FORWARD"
        self.stateTime = 0
        self.ser = serial.Serial(self.port, self.baud)
        self.pub = rospy.Publisher("car_state", String,  queue_size=1)
        self.sub = rospy.Subscriber("car_cmd", Twist, self.ReceiveCmd)

    def ReceiveCmd(self,cmd):
        curForward = cmd.linear.x
        curTurn = cmd.angular.z

        if curForward > 1:
            curForward = 1
        if curForward < -1:
            curForward = -1
        if curTurn > 1:
            curTurn = 1
        if curTurn < -1:
            curTurn = -1
        
        #simulate double click
        if self.state == "FORWARD":
            if curForward < 0:
                self.state = "BACKWARD_CLICK1"
                self.stateTime = time.time()
        elif self.state == "BACKWARD_CLICK1":
            if curForward >= 0:
                self.state = "FORWARD"
            else:
                t = time.time()
                if t - self.stateTime > 0.7:
                    #rospy.loginfo("stateTime: %f" % (t-self.stateTime))
                    self.state = "BACKWARD_PAUSE"
                    self.stateTime = t
        elif self.state == "BACKWARD_PAUSE":
            if curForward >= 0:
                self.state = "FORWARD"
            else:
                t = time.time()
                if t - self.stateTime > 0.4:
                    self.state = "BACKWOARD_CLICK2"
                    #讓車子從低速開始倒退，避免瞬間加速過大
                    curForward = -0.1
                    
        elif self.state == "BACKWOARD_CLICK2":
            if curForward > 0: #車有前進才換到forward，不然留在此state
                self.state = "FORWARD"
            elif curForward < -0.8: #要做到double click的後退訊號會讓車子倒衝太快，這邊把最高速度降低
                curForward = -0.8

        forward = curForward
        if self.state == "BACKWARD_PAUSE":
            forward = 0

        #rospy.loginfo("forward: %f, turn: %f" % (forward,curTurn))
        #send command
        header = 0xFE
        cmd = 0x01
        argNum = 2
        forward = int((forward+1)*0.5*255)
        turn = int((curTurn+1)*0.5*255)
        msg = [header,cmd,argNum,forward,turn]
        
        #compute checksum
        checksum = 0
        for ch in msg:
            checksum += ch
        checksum = checksum%256
        msg.append(checksum)
        #rospy.loginfo("%x %x %x %x %x %x" % (msg[0],msg[1],msg[2],msg[3],msg[4],msg[5]))
        self.ser.write(bytearray(msg))


    def ReceiveState(self,state):
        msg = String()
        msg.data = state
        self.pub.publish(msg)

    def ReadFromSerial(self):
       while not rospy.is_shutdown():
            while self.ser.in_waiting:
                try:
                    msg = self.ser.readline().decode()  # 接收回應訊息並解碼
                    self.ReceiveState(msg)
                except:
                    rospy.loginfo("serial invalid message")


if __name__ == '__main__':
    rospy.init_node('serial_command_node')
    rospy.loginfo("serial_command_node started")
    sc = SerialCommand()
    #use a separate thread to read data from serial
    thread = threading.Thread(target=sc.ReadFromSerial)
    thread.start()
    
    rospy.spin()