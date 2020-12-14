#!/usr/bin/python3.6
# -*- coding: utf-8 -*-
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, CameraInfo
from vapaa_cruiser.msg import objectDetect,objectDetectArray

import torch
import torch.backends.cudnn as cudnn
import numpy as np
from numpy import random

from models.experimental import attempt_load
from utils.general import check_img_size, non_max_suppression, scale_coords, plot_one_box, set_logging
from utils.datasets import letterbox
from utils.torch_utils import select_device

class YoloV5():
    def __init__(self):
        self.br = CvBridge()
        
        self.inFrame = None
        self.image = None
        self.outFrame = None
        self.frameReady = False

        self.weights = rospy.get_param("~weights","yolov5s.pt")
        self.img_size = rospy.get_param("~img_size",640)
        self.conf_thres = rospy.get_param("~conf_thres",0.25)
        self.iou_thres = rospy.get_param("~iou_thres",0.45)
        self.device = rospy.get_param("~device","0")
        self.updateRate = rospy.get_param("~updateRate",30)

        self.subImage = rospy.Subscriber("/camera/color/image_raw/compressed",CompressedImage,self.UpdateFrame)
        self.pubImage = rospy.Publisher("yolov5/detected/compressed",CompressedImage,queue_size=1)

        # Initialize
        set_logging()
        self.device = select_device(str(self.device))
        self.half = self.device.type != 'cpu'  # half precision only supported on CUDA

        # Load model
        self.model = attempt_load(self.weights, map_location=self.device)  # load FP32 model
        self.img_size = check_img_size(self.img_size, s=self.model.stride.max())  # check img_size

        if self.half:
            self.model.half()  # to FP16

        # Get names and colors
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(self.names))]

    def UpdateFrame(self,msg):
        self.frameReady = False
        try:
            self.inFrame = self.br.compressed_imgmsg_to_cv2(msg, "bgr8")
            self.inFrame = letterbox(self.inFrame, new_shape=self.img_size)[0]
            self.image = self.inFrame[:, :, ::-1].transpose(2, 0, 1)
            self.image = np.ascontiguousarray(self.image)
        except CvBridgeError as e:
            print(e)
        self.frameReady = True

    def Run(self):
        rate = rospy.Rate(self.updateRate)
        while not rospy.is_shutdown():
            if self.frameReady:
                # Run inference  
                img = torch.from_numpy(self.image).to(self.device)
                img = img.half() if self.half else img.float()  # uint8 to fp16/32
                img /= 255.0  # 0 - 255 to 0.0 - 1.0

                if img.ndimension() == 3:
                    img = img.unsqueeze(0)

                # Inference
                pred = self.model(img)[0]

                # Apply NMS
                pred = non_max_suppression(pred, self.conf_thres, self.iou_thres)

                # Process detections
                self.outFrame = self.inFrame.copy()
                for i, det in enumerate(pred):  # detections per image
                    if det is not None and len(det):
                        # Rescale boxes from img_size to image size
                        det[:, :4] = scale_coords(img.shape[2:], det[:, :4], self.outFrame.shape).round()

                        for *xyxy, conf, cls in reversed(det):
                            label = '%s %.2f' % (self.names[int(cls)], conf)
                            plot_one_box(xyxy, self.outFrame, label=label, color=self.colors[int(cls)], line_thickness=3)

                imageMsg = self.br.cv2_to_compressed_imgmsg(self.outFrame)
                self.pubImage.publish(imageMsg)

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('yolov5_node')
    rospy.loginfo("yolov5_node started")
    yolov5 = YoloV5()
    yolov5.Run()
