#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from vapaa_cruiser.msg import apriltagDetectArray
from vapaa_cruiser.srv import followTagSetParam,followTagSetParamResponse,followTagGetParam,followTagGetParamResponse
import math

class FollowTag():
    def __init__(self):
        self.state = ""
        self.tagID = rospy.get_param("~tagID",1)
        self.keepDist = rospy.get_param("~keepDist",700)
        self.distTolerance = rospy.get_param("~distTolerance",100)
        self.minAngle = rospy.get_param("~minAngle",-math.pi*0.25)
        self.maxAngle = rospy.get_param("~maxAngle",math.pi*0.25)
        self.speedScale = rospy.get_param("~speedScale",0.0005)
        self.angleScale = rospy.get_param("~angleScale",1.5)

        self.fsmSub = rospy.Subscriber("fsm/state", String, self.ChangeState)
        self.tagDetectSub = rospy.Subscriber("apriltag/detected/info",apriltagDetectArray, self.FollowTag)
        self.cmdPub = rospy.Publisher("car_cmd",Twist, queue_size=1)

        self.getParamSrv = rospy.Service("followTag/getParam", followTagGetParam, self.ServiceGetParam)
        self.setParamSrv = rospy.Service("followTag/setParam", followTagSetParam, self.ServiceSetParam)
        
    def ChangeState(self,msg):
        self.state = msg.data

    def ServiceGetParam(self,request):
        res = followTagGetParamResponse()
        res.tagID = self.tagID
        res.distance = self.keepDist
        res.tolerance = self.distTolerance
        return res

    def ServiceSetParam(self,request):
        self.tagID = request.tagID
        self.keepDist = request.distance
        self.distTolerance = request.tolerance
        rospy.loginfo("update follow tag parameter, tagID=%d, keepDist=%d, distTolerance=%f" % (self.tagID,self.keepDist,self.distTolerance))
        return True
       
    def FollowTag(self,msg):
        if self.state != "FOLLOW_TAG":
            return
        findTag = False
        for tag in msg.tag_array:
            if tag.id == self.tagID:
                findTag = True
                msg = Twist()
                pos = tag.pose.position

                angle = math.atan2(pos.x*self.angleScale,pos.z)
                angle = ((angle-self.minAngle)/(self.maxAngle-self.minAngle)-0.5)*2
                if angle < -1:
                    angle = -1
                elif angle > 1:
                    angle = 1
                msg.angular.z = angle

                speed = pos.z-self.keepDist
                speed = abs(speed)*self.speedScale
                if speed > 1:
                    speed = 1

                if pos.z < self.keepDist-self.distTolerance:    #too close
                    msg.linear.x = -speed
                elif pos.z > self.keepDist+self.distTolerance:  #too far
                    msg.linear.x = speed
                else:   #distance in tolerance
                    msg.linear.x = 0
                #print(msg)
                self.cmdPub.publish(msg)
        if not findTag:
            msg = Twist()
            msg.linear.x = 0
            msg.angular.z = 0
            self.cmdPub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('follow_tag_node')
    rospy.loginfo("follow_tag_node started")

    ft = FollowTag()
    rospy.spin()
