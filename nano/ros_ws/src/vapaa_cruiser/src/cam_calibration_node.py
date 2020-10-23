#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_srvs.srv import Trigger, TriggerResponse
import yaml
import json
import numpy as np
import os
import rospkg

class CamCalibration():
    def __init__(self):
        self.br = CvBridge()
        rospack = rospkg.RosPack()

        self.gridX = rospy.get_param("~gridX",8)
        self.gridY = rospy.get_param("~gridY",6)
        self.gridSize = rospy.get_param("~gridSize",25)
        self.targetW = rospy.get_param("~targetW",640)
        self.targetH = rospy.get_param("~targetH",480)
        self.updateRate = rospy.get_param("~updateRate",30)
        self.outputName = rospy.get_param("~outputName",rospack.get_path("vapaa_cruiser")+"/config/calibration.yml")
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.imageNum = 0
        self.objPtArr = []
        self.imagePtArr = []
        self.objPt = np.zeros((self.gridX*self.gridY,3), np.float32)
        for y in range(self.gridY):
            for x in range(self.gridX):
                self.objPt[y*self.gridX+x] = [x*self.gridSize,y*self.gridSize,0]
        self.inFrame = None
        self.outFrame = None
        self.frameReady = False
        
        self.subImage = rospy.Subscriber("image/compressed",CompressedImage,self.UpdateFrame)
        self.srvAddCorner = rospy.Service("calibration/addCorner", Trigger, self.AddCorner)
        self.srvDoCalib = rospy.Service("calibration/doCalibration", Trigger, self.DoCalibration)
        self.pubImage = rospy.Publisher("calibration/detected/compressed",CompressedImage,queue_size=1)

    def UpdateFrame(self,msg):  #callback裡面不要執行太花時間的功能，不然lag會一直累積
        self.frameReady = False
        try:
            self.inFrame = self.br.compressed_imgmsg_to_cv2(msg, "bgr8")
            self.inFrame = cv2.resize(self.inFrame, (self.targetW, self.targetH), interpolation=cv2.INTER_CUBIC)
        except CvBridgeError as e:
            print(e)

        self.frameReady = True

    def AddCorner(self,request):
        if self.subCorners is None:
            return TriggerResponse(
                success = False,
                message = "no chessboard detected"
            )
        else:
            self.objPtArr.append(self.objPt)
            self.imagePtArr.append(self.subCorners)
            self.imageNum += 1
            return TriggerResponse(
                success = True,
                message = ("add corner # %d" % self.imageNum)
            )

    def DoCalibration(self,request):
        if self.imageNum < 10:
            return TriggerResponse(
                success = False,
                message = "please add at least 10 chessboard images with different positions & angles"
            )
        else:
            ret, mat, distort, rotate, translate = cv2.calibrateCamera(self.objPtArr, self.imagePtArr, (self.targetH,self.targetW),None,None)
            output = {
                "image_width": self.targetW,
                "image_height": self.targetH,
                "camera_matrix": {
                    "rows": mat.shape[0],
                    "cols": mat.shape[1],
                    "data": mat.flatten().tolist()
                },
                "distortion_coefficients": {
                    "rows": distort.shape[0],
                    "cols": distort.shape[1],
                    "data": distort.flatten().tolist()
                },
            }

            dirName = os.path.dirname(self.outputName)
            if not os.path.exists(dirName):
		os.makedirs(dirName)
            with open(self.outputName, "w") as f:
                yaml.dump(output, f)

            #reset 
            self.imageNum = 0
            self.objPtArr = []
            self.imagePtArr = []

            msg = "save calibration result to %s\ndata: %s" % (os.getcwd()+"/"+self.outputName,json.dumps(output))
            return TriggerResponse(
                success = True,
                message = msg
            )

    def Run(self):
        rate = rospy.Rate(self.updateRate)
        while not rospy.is_shutdown():
            if self.frameReady:
                gray = cv2.cvtColor(self.inFrame, cv2.COLOR_BGR2GRAY)
                self.outFrame = self.inFrame.copy()
                ret, corners = cv2.findChessboardCorners(gray, (self.gridX,self.gridY),None)
                
                if ret == True:
                    self.subCorners = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),self.criteria)
                    self.outFrame = cv2.drawChessboardCorners(self.outFrame, (self.gridX,self.gridY), self.subCorners,ret)
                else:
                    self.subCorners = None

                imageMsg = self.br.cv2_to_compressed_imgmsg(self.outFrame)
                self.pubImage.publish(imageMsg)
            rate.sleep() 


if __name__ == '__main__':
    rospy.init_node('cam_calibration_node')
    rospy.loginfo("cam_calibration_node started")
    cc = CamCalibration()
    cc.Run()
