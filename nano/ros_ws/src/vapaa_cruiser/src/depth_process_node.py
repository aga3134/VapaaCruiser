#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage,Image, CameraInfo
from vapaa_cruiser.msg import ObjectDetect,ObjectDetectArray
import numpy as np
from numpy import random

class DepthProcess():
    def __init__(self):
        self.br = CvBridge()
        self.pubImage = rospy.Publisher("depth_process/image/compressed",CompressedImage,queue_size=1)
        self.subImage = rospy.Subscriber("camera/aligned_depth_to_color/image_raw",Image,self. RecieveDepth)
        self.subYolov4 = rospy.Subscriber("yolov4/object",ObjectDetectArray,self. RecieveYolov4)
        self.subYolov5 = rospy.Subscriber("yolov5/object",ObjectDetectArray,self. RecieveYolov5)
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
                #realsense ros publish 16bit image with  unit 1mm, scale image value for more  clear  visualization
                self.outFrame = cv2.cvtColor((self.inFrame/20), cv2.COLOR_GRAY2BGR)

                objArr = None
                if self.yolov4Obj is not None:
                    objArr = self.yolov4Obj.object_array
                elif self.yolov5Obj is not None:
                    objArr = self.yolov5Obj.object_array

                if objArr is not None:
                    for obj in objArr:
                        centerX = int((obj.corner[0].x+obj.corner[1].x+obj.corner[2].x+obj.corner[3].x)*0.25)
                        centerY = int((obj.corner[0].y+obj.corner[1].y+obj.corner[2].y+obj.corner[3].y)*0.25)
                        depth = None
                        shape = self.inFrame.shape
                        if centerY >= 0 and centerY < shape[0] and centerX >= 0 and centerX < shape[1]:
                            depth = self.inFrame[centerY,centerX]
                        #print(obj.name)
                        cv2.rectangle(self.outFrame, 
                            (int(obj.corner[0].x), int(obj.corner[0].y)), 
                            (int(obj.corner[2].x), int(obj.corner[2].y)),
                            self.colors[obj.id], 2)

                        #draw label
                        scale = 0.3
                        textSize = cv2.getTextSize(obj.name, 0, fontScale=scale, thickness=1)[0]
                        c1 = (int(obj.corner[0].x), int(obj.corner[0].y))
                        c2 = (c1[0] + textSize[0], c1[1] - textSize[1]-3)
                        cv2.rectangle(self.outFrame, c1, c2, self.colors[obj.id], -1, cv2.LINE_AA)  # filled
                        cv2.putText(self.outFrame, obj.name, (c1[0],c1[1]-2), 0, scale, [255, 255, 255], thickness=1, lineType=cv2.LINE_AA)
                        if depth is not None:
                            cv2.putText(self.outFrame,str(depth)+"mm", (int(obj.corner[3].x+3),int(obj.corner[3].y-3)),0,scale,[255,0,0],thickness=1,lineType=cv2.LINE_AA)

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
