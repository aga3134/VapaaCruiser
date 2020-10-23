#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_srvs.srv import Trigger, TriggerResponse
from vapaa_cruiser.srv import imageStoreUpload
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
        self.triggerSaveFront = False
        self.triggerSaveSide = False
        self.triggerUploadFront = False
        self.triggerUploadSide = False
        self.uploadFrontInfo = None
        self.uploadSideInfo = None
        
        self.subFront = rospy.Subscriber("image/compressed",CompressedImage,self.RecieveFrontImage)
        self.subSide = rospy.Subscriber("/camera/color/image_raw/compressed",CompressedImage,self.RecieveSideImage)
        self.srvSaveFront = rospy.Service("imageStore/saveFront", Trigger, self.SaveFront)
        self.srvSaveSide = rospy.Service("imageStore/saveSide", Trigger, self.SaveSide)
        self.srvUploadFront = rospy.Service("imageStore/uploadFront", imageStoreUpload, self.UploadFront)
        self.srvUploadSide = rospy.Service("imageStore/uploadSide", imageStoreUpload, self.UploadSide)

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
        if not self.triggerSaveFront and not self.triggerUploadFront:
            return
        try:
            frame = self.br.compressed_imgmsg_to_cv2(msg, "bgr8")
            if self.triggerSaveFront:
                self.SaveImage(frame,self.savePath)
            if self.triggerUploadFront:
                self.UploadImage(frame,self.uploadFrontInfo)
        except CvBridgeError as e:
            print(e)

        self.triggerSaveFront = False
        self.triggerUploadFront = False

    def RecieveSideImage(self,msg):
        if not self.triggerSaveSide and not self.triggerUploadSide:
            return
        try:
            frame = self.br.compressed_imgmsg_to_cv2(msg, "bgr8")
            if self.triggerSaveSide:
                self.SaveImage(frame,self.savePath)
            if self.triggerUploadSide:
                self.UploadImage(frame,self.uploadSideInfo)
        except CvBridgeError as e:
            print(e)

        self.triggerSaveFront = False
        self.triggerUploadFront = False

    def SaveFront(self,request):
        self.triggerSaveFront = True
        return TriggerResponse(
            success = True,
            message = "schedule to save front image"
        )

    def SaveSide(self,request):
        self.triggerSaveSide = True
        return TriggerResponse(
            success = True,
            message = "schedule to save side image"
        )

    def UploadFront(self,request):
        try:
            self.uploadFrontInfo = json.parse(request.info)
            self.triggerUploadFront = True
            return TriggerResponse(
                success = True,
                message = "schedule to upload front image"
            )
        except:
            return TriggerResponse(
                success = False,
                message = "parse info fail"
            )


    def UploadSide(self,request):
        try:
            self.uploadSideInfo = json.parse(request.info)
            self.triggerUploadSide = True
            return TriggerResponse(
                success = True,
                message = "schedule to upload side image"
            )
        except:
            return TriggerResponse(
                success = False,
                message = "parse info fail"
            )

if __name__ == '__main__':
    rospy.init_node('image_store_node')
    rospy.loginfo("image_store_node started")
    imageStore = ImageStore()
    rospy.spin()
