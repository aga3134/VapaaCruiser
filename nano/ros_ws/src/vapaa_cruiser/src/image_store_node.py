#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_srvs.srv import Trigger, TriggerResponse
from vapaa_cruiser.srv import TriggerWIthInfo
import json
import numpy as np
import os
import rospkg
import datetime
import base64
import requests
import pytz

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
        self.srvImageStoreFront = rospy.Service("imageStore/front", TriggerWithInfo, self.ImageStoreFront)
        self.srvImageStoreFront = rospy.Service("imageStore/side", TriggerWithInfo, self.ImageStoreSide)

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

        url = self.uploadHost+"/api/upload-image"
        data = {}
        data["dataset"] = info["dataset"]
        data["apiKey"] = info["apiKey"]
        if "lat" in info and "lng" in info:
            data["lat"] = info["lat"]
            data["lng"] = info["lng"]
        data["remark"] = "upload from vapaa_cruiser"
        data["dataTime"] = datetime.datetime.now(pytz.utc)
        data["uploadImage"] = base64.b64encode(img_encoded)

        response = requests.post(url,data=data)
        print(response.text)

    def RecieveFrontImage(self,msg):
        if not self.triggerFront:
            return
        self.triggerFront = False
        try:
            frame = self.br.compressed_imgmsg_to_cv2(msg, "bgr8")
            if self.storeFrontInfo["saveImage"]:
                self.SaveImage(frame,self.savePath)
            if self.storeFrontInfo["uploadImage"]:
                self.UploadImage(frame,self.storeFrontInfo)
        except CvBridgeError as e:
            print(e)

    def RecieveSideImage(self,msg):
        if not self.triggerSide:
            return
        self.triggerSide = False
        try:
            frame = self.br.compressed_imgmsg_to_cv2(msg, "bgr8")
            if self.storeSideInfo["saveImage"]:
                self.SaveImage(frame,self.savePath)
            if self.storeSideInfo["uploadImage"]:
                self.UploadImage(frame,self.storeSideInfo)
        except CvBridgeError as e:
            print(e)

    def ImageStoreFront(self,request):
        try:
            self.storeFrontInfo = json.loads(request.info)
            self.triggerFront = True
            return True
        except:
            return False

    def ImageStoreSide(self,request):
        try:
            self.storeSideInfo = json.loads(request.info)
            self.triggerSide = True
            return True
        except:
            return False

if __name__ == '__main__':
    rospy.init_node('image_store_node')
    rospy.loginfo("image_store_node started")
    imageStore = ImageStore()
    rospy.spin()
