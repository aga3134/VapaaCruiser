#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
import tf2_ros
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
import geometry_msgs.msg
from vapaa_cruiser.msg import mapPose
import math
import numpy as np

class AutoNavigation():
    def __init__(self):
        self.updateRate = rospy.get_param("~updateRate",30)

        self.subState = rospy.Subscriber("car_state",String,self.UpdateState)
        self.pubMapPose = rospy.Publisher("map_pose",mapPose,queue_size=1)

        self.navState = "pause"
        self.navLoop = False
        self.pathID = ""
        self.lat = -9999
        self.lng = -9999
        self.usDist = {"LF":-1,"F":-1,"RF":-1,"LB":-1,"B":-1,"RB":-1}
        self.Trans = {  #for odom to gps transform computation
            #y依angle旋轉後乘scale，加上offsetY即為lat
            #x依angle旋轉後乘scale，除以cos(lat)調整，再加上offsetX即為lng
            "maxNum": 100,
            "mappingArr": [],   #存原始資料
            "dxSum": 0,
            "dySum": 0,
            "dLatSum": 0,
            "dLngSum": 0,
            "angle": 0,
            "scale": 0,
            "offsetX": 0,
            "offsetY": 0,
            "inited": False
        }
        self.startNavSrv = rospy.Service("auto_navigation/start", Trigger, self.ServiceStartNav)
        self.stopNavSrv = rospy.Service("auto_navigation/stop", Trigger, self.ServiceStopNav)
        self.pauseNavSrv = rospy.Service("auto_navigation/pause", Trigger, self.ServicePauseNav)
        self.setLoopSrv = rospy.Service("auto_navigation/setLoop", SetBool, self.ServiceSetLoop)

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        
        #realsense向左看，對z軸旋轉-90度才是車體正前方
        #tf tree: /map -> /odom -> /camera_link -> /car
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "camera_link"
        t.child_frame_id = "car"
        t.transform.translation.x = 0
        t.transform.translation.y = 0
        t.transform.translation.z = 0
        quat = tf.transformations.quaternion_from_euler(0,0,-math.pi*0.5)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        broadcaster.sendTransform(t)

    def UpdateState(self,msg):
        data = msg.data.split(",")
        self.lat = data[0]
        self.lng = data[1]
        self.usDist["LF"] = data[2]
        self.usDist["F"] = data[3]
        self.usDist["RF"] = data[4]
        self.usDist["LB"] = data[5]
        self.usDist["B"] = data[6]
        self.usDist["RB"] = data[7]

    def ServiceStartNav(self,request):
        self.navState = "start"

    def ServiceStopNav(self,request):
        self.navState = "stop"

    def ServicePauseNav(self,request):
        self.navState = "pause"

    def ServiceSetLoop(self,request):
        self.navLoop = request.data

    def UpdateOdomToGPSTransform(self,x,y,lat,lng):
        #假設地球是正圓形，經度隨距離變化量隨緯度越高變化越快
        #將經度變化量乘以cos(緯度)消除緯度的影響
        #此時經度、緯度變化360度的距離 = 2pi*地球半徑
        r = 6378137 #地球半徑(m)
        self.Trans["scale"] = 180.0/(r*math.pi)

        dSize = len(self.Trans["mappingArr"])
        if dSize < 2:
            self.Trans["mappingArr"].append({
                "x": x, "y": y, "lat": lat, "lng": lng,
                "dx": 0, "dy": 0, "dLat":0, "dLng":0
            })
        else:  #compute difference
            newPt = {
                "x": x, "y":y, "lat":lat, "lng":lng
            }
            lastPt = self.Trans["mappingArr"][dSize-1]
            newPt["dx"] = newPt["x"] - lastPt["x"]
            newPt["dy"] = newPt["y"] - lastPt["y"]
            if newPt["dx"]*newPt["dx"]+newPt["dy"]*newPt["dy"] < 0.01:  #距離太近，不加入計算
                return
            newPt["dLat"] = (newPt["lat"] - lastPt["lat"])
            newPt["dLng"] = (newPt["lng"] - lastPt["lng"])
            newPt["dLng"] *= math.cos(lastPt["lat"])
            self.Trans["mappingArr"].append(newPt)
            self.Trans["dxSum"] += newPt["dx"]
            self.Trans["dySum"] += newPt["dy"]
            self.Trans["dLatSum"] += newPt["dLat"]
            self.Trans["dLngSum"] += newPt["dLng"]

        if len(self.Trans["mappingArr"]) > self.Trans["maxNum"]:    #moving window
            firstPt = self.Trans["mappingArr"][0]
            self.Trans["dxSum"] -= firstPt["dx"]
            self.Trans["dySum"] -= firstPt["dy"]
            self.Trans["dLatSum"] -= firstPt["dLat"]
            self.Trans["dLngSum"] -= firstPt["dLng"]
            self.Trans["mappingArr"].pop(0)
        dSize = len(self.Trans["mappingArr"])

        #compute angle between xy & latlng
        xyNorm = math.sqrt(self.Trans["dxSum"]*self.Trans["dxSum"]+self.Trans["dySum"]*self.Trans["dySum"])
        if xyNorm == 0:
            return
        latLngNorm = math.sqrt(self.Trans["dLatSum"]*self.Trans["dLatSum"]+self.Trans["dLngSum"]*self.Trans["dLngSum"])
        if latLngNorm == 0:
            return
        xyVec = [self.Trans["dxSum"]/xyNorm, self.Trans["dySum"]/xyNorm]
        latLngVec = [self.Trans["dLngSum"]/latLngNorm, self.Trans["dLatSum"]/latLngNorm]
        dot = xyVec[0]*latLngVec[0]+xyVec[1]*latLngVec[1]
        cross = xyVec[0]*latLngVec[1]-xyVec[1]*latLngVec[0]
        self.Trans["angle"] = math.atan2(cross,dot)

        #compute offset between xy & latlng
        cosAngle = dot
        sinAngle = cross
        offsetXSum = 0
        offsetYSum = 0
        for pt in self.Trans["mappingArr"]:
            x = self.Trans["scale"]*(cosAngle*pt["x"]-sinAngle*pt["y"])
            y = self.Trans["scale"]*(sinAngle*pt["x"]+cosAngle*pt["y"])
            cosLat = math.cos(pt["lat"])
            if cosLat == 0: #極地
                continue
            x = x/cosLat
            offsetXSum += (pt["lng"]-x)
            offsetYSum += (pt["lat"]-y)
        self.Trans["offsetX"] = offsetXSum/dSize
        self.Trans["offsetY"] = offsetYSum/dSize

        #print([self.Trans["scale"],self.Trans["angle"],self.Trans["offsetX"],self.Trans["offsetY"]])

    def XYToLatLng(self,x,y):
        cosAngle = math.cos(self.Trans["angle"])
        sinAngle = math.sin(self.Trans["angle"])
        lat = self.Trans["scale"]*(sinAngle*x+cosAngle*y)+self.Trans["offsetY"]
        lng = self.Trans["scale"]*(cosAngle*x-sinAngle*y)/math.cos(lat)+self.Trans["offsetX"]
        return {"lat":lat, "lng":lng}

    def LatLngToXY(self,lat,lng):
        x = (lat-self.Trans["offsetY"])/self.Trans["scale"]
        y = (lng-self.Trans["offsetX"])/self.Trans["scale"]
        cosAngle = math.cos(-self.Trans["angle"])
        sinAngle = math.sin(-self.Trans["angle"])
        rotX = sinAngle*x+cosAngle*y
        rotY = cosAngle*x-sinAngle*y
        return {"x":rotX, "y":rotY}

    def GenFakeLatLng(self,x,y): #generate fake lat lng for testing
        offsetX = 121
        offsetY = 23
        angle = -180.0*math.pi/180.0
        scale = 180.0/(6378137*math.pi)
        #print([scale,angle,offsetX,offsetY])

        cosAngle = math.cos(angle)
        sinAngle = math.sin(angle)
        self.lat = scale*(sinAngle*x+cosAngle*y)+offsetY
        self.lng = scale*(cosAngle*x-sinAngle*y)/math.cos(self.lat)+offsetX
        #print([x,y,self.lat,self.lng])

    def Run(self):
        rate = rospy.Rate(self.updateRate)
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            try:
                #get car pose from odometry
                pose = self.tfBuffer.lookup_transform("car", "map", rospy.Time()).transform
                elapse = (rospy.Time.now()-start)
                t = elapse.secs+elapse.nsecs*1e-9
                #pose.translation.x = t
                #pose.translation.y = t
                self.GenFakeLatLng(pose.translation.x,pose.translation.y)
                if not self.Trans["inited"]:
                    self.Trans["angle"] = 0
                    self.Trans["offsetX"] = self.lng
                    self.Trans["offsetY"] = self.lat
                    self.Trans["inited"] = True
                self.UpdateOdomToGPSTransform(pose.translation.x,pose.translation.y,self.lat,self.lng)
                
                latlng = self.XYToLatLng(pose.translation.x,pose.translation.y)
                q = [pose.rotation.x,pose.rotation.y,pose.rotation.z,pose.rotation.w]
                angle = tf.transformations.euler_from_quaternion(q)
                mapPoseMsg = mapPose()
                mapPoseMsg.lat = latlng["lat"]
                mapPoseMsg.lng = latlng["lng"]
                mapPoseMsg.angle = (angle[2]+self.Trans["angle"])*180/math.pi
                self.pubMapPose.publish(mapPoseMsg)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                #rospy.logwarn("auto_navigation lookup transform error")
                pass
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('auto_navigation_node')
    rospy.loginfo("auto_navigation_node started")
    an = AutoNavigation()
    an.Run()
