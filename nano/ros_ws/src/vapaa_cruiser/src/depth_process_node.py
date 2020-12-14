#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage,Image, CameraInfo
from vapaa_cruiser.msg import objectDetect,objectDetectArray
import numpy as np

class DepthProcess():
    def __init__(self):
        self.br = CvBridge()
        self.pubImage = rospy.Publisher("depth_process/image/compressed",CompressedImage,queue_size=1)
        self.subImage = rospy.Subscriber("camera/aligned_depth_to_color/image_raw",Image,self. RecieveDepth)
        self.inFrame = None
        self.outFrame = None
        self.rate = rospy.get_param("~rate",30)

    def RecieveDepth(self,msg):
        try:
            self.inFrame = self.br.imgmsg_to_cv2(msg)
        except CvBridgeError as e:
            print(e)

    def Run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.inFrame is not None:
                if self.outFrame is None:
                    h, w = self.inFrame.shape[:2]
                    self.outFrame = np.zeros((h, w, 3), dtype=np.uint8)
                    #print(self.outFrame.shape)

                #realsense ros publish 16bit image with  unit mm, scale image value for more  clear  visualization
                self.outFrame = (self.inFrame/20).astype('uint8')
            
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
