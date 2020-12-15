#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage,Image, CameraInfo
from vapaa_cruiser.msg import objectDetect,objectDetectArray
import numpy as np
from numpy import random

class DepthProcess():
    def __init__(self):
        self.br = CvBridge()
        self.pubImage = rospy.Publisher("depth_process/image/compressed",CompressedImage,queue_size=1)
        self.subImage = rospy.Subscriber("camera/aligned_depth_to_color/image_raw",Image,self. RecieveDepth)
        self.subYolov4 = rospy.Subscriber("yolov4/object",objectDetectArray,self. RecieveYolov4)
        self.subYolov5 = rospy.Subscriber("yolov5/object",objectDetectArray,self. RecieveYolov5)
        self.inFrame = None
        self.outFrame = None
        self.yolov4Obj = None
        self.yolov5Obj = None
        self.rate = rospy.get_param("~rate",30)
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(100)]

    def RecieveDepth(self,msg):
        try:
            self.inFrame = self.br.imgmsg_to_cv2(msg)
        except CvBridgeError as e:
            print(e)

    def RecieveYolov4(self,msg):
        self.yolov4Obj = msg
        self.yolov5Obj = None
    
    def RecieveYolov5(self,msg):
        self.yolov5Obj = msg
        self.yolov4Obj = None

    def Run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.inFrame is not None:
                #realsense ros publish 16bit image with  unit mm, scale image value for more  clear  visualization
                self.outFrame = cv2.cvtColor((self.inFrame/20), cv2.COLOR_GRAY2BGR)

                objArr = None
                if self.yolov4Obj is not None:
                    objArr = self.yolov4Obj.object_array
                elif self.yolov5Obj is not None:
                    objArr = self.yolov5Obj.object_array

                if objArr is not None:
                    for obj in objArr:
                        #print(obj.name)
                        cv2.rectangle(self.outFrame, 
                            (int(obj.corner[0].x), int(obj.corner[0].y)), 
                            (int(obj.corner[2].x), int(obj.corner[2].y)),
                            self.colors[obj.id], 2)
            
                imageMsg = self.br.cv2_to_compressed_imgmsg(self.outFrame)
                self.pubImage.publish(imageMsg)
                
                #cv2.imshow('frame',self.outFrame)
                #cv2.waitKey(1)
            rate.sleep() 

if __name__ == '__main__':
    rospy.init_node('depth_process_node')
    rospy.loginfo("depth_process_node started")
    dp = DepthProcess()
    dp.Run()
