#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import Empty
import datetime
import os
import yaml

class CamCapture():
    def __init__(self):
        self.K = None
        self.D = None
        configFile = rospy.get_param("~configFile","config/calibration.yml")
        if os.path.isfile(configFile):
            with open(configFile) as f:
                config = yaml.load(f)
                self.width = config["image_width"]
                self.height = config["image_height"]
                self.K = config["camera_matrix"]["data"]
                self.D = config["distortion_coefficients"]["data"]
        else:
            self.width = rospy.get_param("~width",640)
            self.height = rospy.get_param("~height",480)

        self.camID = rospy.get_param("~camID",None)
        self.rate = rospy.get_param("~rate",30)
        self.flipMode = rospy.get_param("~flipMode",0)
        self.frameID = rospy.get_param("~frameID","camera") #camera coordinate frame
        
        
        if self.camID is None:
            self.cap = cv2.VideoCapture(self.gstreamer_pipeline(), cv2.CAP_GSTREAMER)
            rospy.loginfo("use gstreamer, w=%d, h=%d, rate=%f" % (self.width,self.height,self.rate))
        else:
            self.cap = cv2.VideoCapture(self.camID)
            rospy.loginfo("use camera id=%d, w=%d, h=%d, rate=%f" % (self.camID,self.width,self.height,self.rate))

        self.br = CvBridge()
        self.pubImage = rospy.Publisher("image/compressed",CompressedImage,queue_size=1)
        self.pubInfo = rospy.Publisher("image/info",CameraInfo,queue_size=1)
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
            #publish camera image
            imageMsg = self.br.cv2_to_compressed_imgmsg(self.frame)
            self.pubImage.publish(imageMsg)
            #publish camera info
            infoMsg = CameraInfo()
            infoMsg.header.frame_id = self.frameID
            infoMsg.header.stamp = rospy.Time.now()
            infoMsg.width = self.width
            infoMsg.height = self.height
            infoMsg.distortion_model = "plumb_bob"
            if self.D is not None:
                infoMsg.D = self.D
            if self.K is not None:
                infoMsg.K = self.K
            self.pubInfo.publish(infoMsg)

            #cv2.imshow('frame',self.frame)
            #cv2.waitKey(1)
            rate.sleep() 

if __name__ == '__main__':
    rospy.init_node('cam_capture_node')
    rospy.loginfo("cam_capture_node started")
    cam = CamCapture()
    cam.Run()
