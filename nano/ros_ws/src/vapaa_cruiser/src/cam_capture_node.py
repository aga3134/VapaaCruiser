#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import Empty
import datetime
import os

class CamCapture():
    def __init__(self):
        self.camID = rospy.get_param("~camID",0)
        self.width = rospy.get_param("~width",640)
        self.height = rospy.get_param("~height",480)
        self.rate = rospy.get_param("~rate",30)
        self.flipMode = rospy.get_param("~flipMode",0)
        rospy.loginfo("use camera id=%d, w=%d, h=%d, rate=%f" % (self.camID,self.width,self.height,self.rate))
        
        self.cap = cv2.VideoCapture(self.gstreamer_pipeline(), cv2.CAP_GSTREAMER)
        self.br = CvBridge()
        self.pub = rospy.Publisher("image/compressed",CompressedImage,queue_size=1)
        self.sub = rospy.Subscriber("image/capture",Empty,self.CaptureImage)
        self.savePath = rospy.get_param("~savePath","captureImage/")
        self.frame = None

    def gstreamer_pipeline(self):
        return (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), "
            "width=(int)%d, height=(int)%d, "
            "format=(string)NV12, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (
                self.width,
                self.height,
                self.rate,
                self.flipMode,
                self.width,
                self.height,
            )
        )

    def CaptureImage(self,msg):
        if self.frame is not None:
            if not os.path.exists(self.savePath):
                os.makedirs(self.savePath)

            now = datetime.datetime.now()
            filename = self.savePath+now.strftime("%Y-%m-%d_%H-%M-%S")+".jpg"
            rospy.loginfo("save image "+filename)
            cv2.imwrite(filename, self.frame)

    def Run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            ret, self.frame = self.cap.read()
            msg = self.br.cv2_to_compressed_imgmsg(self.frame)
            self.pub.publish(msg)
            #cv2.imshow('frame',self.frame)
            #cv2.waitKey(1)
            rate.sleep() 

if __name__ == '__main__':
    rospy.init_node('cam_capture_node')
    rospy.loginfo("cam_capture_node started")
    cam = CamCapture()
    cam.Run()
