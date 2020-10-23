#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_srvs.srv import Trigger, TriggerResponse
from vapaa_cruiser.srv import imageStoreInfo
import json
import numpy as np
import os
import rospkg
import datetime

class ImageStore():
    def __init__(self):
        self.br = CvBridge()
        rospack = rospkg.RosPack()

        self.savePath = rospy.get_param("~savePath",rospack.get_path("vapaa_cruiser")+"/save/")
        self.uploadHost = rospy.get_param("~uploadHost","https://commutag.agawork.tw")
        self.triggerFront = False
        self.triggerSide = False
        self.storeFrontInfo = None
        self.storeSideInfo = None
        
        self.subFront = rospy.Subscriber("image/compressed",CompressedImage,self.RecieveFrontImage)
        self.subSide = rospy.Subscriber("/camera/color/image_raw/compressed",CompressedImage,self.RecieveSideImage)
        self.srvImageStoreFront = rospy.Service("imageStore/front", imageStoreInfo, self.ImageStoreFront)
        self.srvImageStoreFront = rospy.Service("imageStore/side", imageStoreInfo, self.ImageStoreSide)

    def SaveImage(self,frame,savePath):
        if not os.path.exists(savePath):
            os.makedirs(savePath)
        now = datetime.datetime.now()
        filename = savePath+now.strftime("%Y-%m-%d_%H-%M-%S")+".jpg"
        rospy.loginfo("save image "+filename)
        cv2.imwrite(filename, frame)

    def UploadImage(self,frame,info):
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        _, img_encoded = cv2.imencode(".jpg",frame,encode_param)

        url = host+"/api/upload-image"
        data = {}
        data["dataset"] = info["dataset"]
        data["apiKey"] = info["apiKey"]
        data["lat"] = info["lat"]
        data["lng"] = info["lng"]
        data["remark"] = "upload from vapaa_cruiser"
        data["dataTime"] = datetime.datetime.now()
        data["uploadImage"] = base64.b64encode(img_encoded)

        response = requests.post(url,data=data)
        print(response.text)

    def RecieveFrontImage(self,msg):
        if not self.triggerFront:
            return
        try:
            frame = self.br.compressed_imgmsg_to_cv2(msg, "bgr8")
            if self.storeFrontInfo["saveImage"]:
                self.SaveImage(frame,self.savePath)
            if self.storeFrontInfo["uploadImage"]:
                self.UploadImage(frame,self.uploadFrontInfo)
        except CvBridgeError as e:
            print(e)
        self.triggerFront = False

    def RecieveSideImage(self,msg):
        if not self.triggerSide:
            return
        try:
            frame = self.br.compressed_imgmsg_to_cv2(msg, "bgr8")
            if self.storeSideInfo["saveImage"]:
                self.SaveImage(frame,self.savePath)
            if self.storeSideInfo["uploadImage"]:
                self.UploadImage(frame,self.uploadSideInfo)
        except CvBridgeError as e:
            print(e)
        self.triggerSide = False

    def ImageStoreFront(self,request):
        try:
            self.storeFrontInfo = json.parse(request.info)
            print(self.storeFrontInfo)
            self.triggerFront = True
            return True
        except:
            return False

    def ImageStoreSide(self,request):
        try:
            self.storeSideInfo = json.parse(request.info)
            print(self.storeSideInfo)
            self.triggerSide = True
            return True
        except:
            return False

if __name__ == '__main__':
    rospy.init_node('image_store_node')
    rospy.loginfo("image_store_node started")
    imageStore = ImageStore()
    rospy.spin()
