#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import apriltag
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, CameraInfo
from vapaa_cruiser.msg import apriltagDetectArray, apriltagDetect
import tf

class ApriltagDetector():
    def __init__(self):
        self.br = CvBridge()
        self.detector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11") )
        self.camParam = None
        self.inFrame = None
        self.outFrame = None
        self.frameID = None
        self.frameReady = False
        self.tagSize = rospy.get_param("~tagSize",30)
        self.updateRate = rospy.get_param("~updateRate",30)

        self.subImage = rospy.Subscriber("image/compressed",CompressedImage,self.UpdateFrame)
        self.subInfo = rospy.Subscriber("image/info",CameraInfo,self.UpdateCameraInfo)
        self.pubImage = rospy.Publisher("apriltag/detected/compressed",CompressedImage,queue_size=1)
        self.pubInfo = rospy.Publisher("apriltag/detected/info",apriltagDetectArray,queue_size=1)

    def UpdateCameraInfo(self,msg):
        self.camParam = [msg.K[0], msg.K[4], msg.K[2], msg.K[5]]
        self.frameID = msg.header.frame_id

    def UpdateFrame(self,msg):
        self.frameReady = False
        try:
            self.inFrame = self.br.compressed_imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.frameReady = True

    def Run(self):
        rate = rospy.Rate(self.updateRate)
        while not rospy.is_shutdown():
            if self.frameReady:
                gray = cv2.cvtColor(self.inFrame, cv2.COLOR_BGR2GRAY)
                self.outFrame = self.inFrame.copy()
                infoMsg = apriltagDetectArray()
                infoMsg.header.frame_id = self.frameID
                infoMsg.header.stamp = rospy.Time.now()

                tags = self.detector.detect(gray)
                for tag in tags:
                    cv2.line(self.outFrame, tuple(tag.corners[0].astype(int)), tuple(tag.corners[1].astype(int)), (0, 255, 0), 2)
                    cv2.line(self.outFrame, tuple(tag.corners[1].astype(int)), tuple(tag.corners[2].astype(int)), (0, 0, 255), 2)
                    cv2.line(self.outFrame, tuple(tag.corners[2].astype(int)), tuple(tag.corners[3].astype(int)), (255, 0, 0), 2)
                    cv2.line(self.outFrame, tuple(tag.corners[3].astype(int)), tuple(tag.corners[0].astype(int)), (255, 0, 0), 2)

                    tagInfo = apriltagDetect()
                    tagInfo.id = tag.tag_id
                    tagText = str(tag.tag_id)
                    for i in range(4):
                        tagInfo.corner[i].x = tag.corners[i][0]
                        tagInfo.corner[i].y = tag.corners[i][1]

                    if self.camParam is not None:
                        pose, e0, e1 = self.detector.detection_pose(tag,self.camParam,self.tagSize)
                        p = tf.transformations.translation_from_matrix(pose)
                        o = tf.transformations.quaternion_from_matrix(pose)
                        tagInfo.pose.position.x = p[0]
                        tagInfo.pose.position.y = p[1]
                        tagInfo.pose.position.z = p[2]
                        tagInfo.pose.orientation.x = o[0]
                        tagInfo.pose.orientation.y = o[1]
                        tagInfo.pose.orientation.z = o[2]
                        tagInfo.pose.orientation.w = o[3]

                        tagText += ": "+str(int(p[2]))+"mm"

                    infoMsg.tag_array.append(tagInfo)

                    cv2.putText(self.outFrame, tagText, tuple(tag.corners[0].astype(int)), 0, 0.3, [255, 0, 0], thickness=1, lineType=cv2.LINE_AA)
                imageMsg = self.br.cv2_to_compressed_imgmsg(self.outFrame)
                self.pubImage.publish(imageMsg)
    
                self.pubInfo.publish(infoMsg)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('apriltag_node')
    rospy.loginfo("apriltag_node started")
    ad = ApriltagDetector()
    ad.Run()
