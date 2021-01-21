#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
import tf2_ros
from std_msgs.msg import String
from std_srvs.srv import Trigger,TriggerResponse,SetBool,SetBoolResponse
from geometry_msgs.msg import Twist,TransformStamped
from vapaa_cruiser.msg import NavState
from vapaa_cruiser.srv import TriggerWithInfo,TriggerWithInfoResponse
import math
import numpy as np
import json
import sqlite3
import rospkg

class AutoNavigation():
    def __init__(self):
        self.updateRate = rospy.get_param("~updateRate",30)
        rospack = rospkg.RosPack()
        self.dbFile = rospy.get_param("~dbFile",rospack.get_path("vapaa_cruiser")+"/../../../web/vapaa_cruiser.db")
        self.useFakeGPS = rospy.get_param("~useFakeGPS", False)
        self.fakeInitLat = rospy.get_param("~fakeInitLat", 23)
        self.fakeInitLng = rospy.get_param("~fakeInitLng", 121)
        self.fakeInitAngle = rospy.get_param("~fakeInitAngle", 180)

        self.subCarState = rospy.Subscriber("car_state",String,self.UpdateState)
        self.pubNavState = rospy.Publisher("nav_state",NavState,queue_size=1)
        self.pubCarCmd = rospy.Publisher("car_cmd", Twist, queue_size=1)

        self.navState = "stop"
        self.navPause = False
        self.navLoop = False
        self.curPath = None
        self.targetIndex = 0
        self.drive = {
            "targetForward": 0, "targetTurn": 0,
            "curForward": 0, "curTurn": 0,
            "errForward": 0, "errTurn": 0,
            "errSumForward": 0, "errSumTurn": 0,
            "forwardP": 1, "forwardI": 0, "forwardD": 0,
            "turnP": 1, "turnI": 0, "turnD": 0,
            "adjustByUS": True
        }
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
        self.srvStartNav = rospy.Service("autoNavigation/start", TriggerWithInfo, self.ServiceStartNav)
        self.srvStopNav = rospy.Service("autoNavigation/stop", Trigger, self.ServiceStopNav)
        self.srvPauseNav = rospy.Service("autoNavigation/pause", SetBool, self.ServiceSetPause)
        self.srvSetLoop = rospy.Service("autoNavigation/setLoop", SetBool, self.ServiceSetLoop)

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        
        #realsense向左看，對z軸旋轉-90度才是車體正前方
        #tf tree: /map -> /odom -> /camera_link -> /car
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        t = TransformStamped()
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
        self.usDist["FL"] = data[2]
        self.usDist["F"] = data[3]
        self.usDist["FR"] = data[4]
        self.usDist["BL"] = data[5]
        self.usDist["B"] = data[6]
        self.usDist["BR"] = data[7]

    def ServiceStartNav(self,request):
        info = json.loads(request.info)
        conn = sqlite3.connect(self.dbFile)
        c = conn.cursor()
        cmd = "SELECT * FROM NavigationPath WHERE id='%s';" % (info["id"])
        row = c.execute(cmd).fetchone()
        conn.close()
        if row is None:
            return TriggerWithInfoResponse(
                success=False,
                message="path not exist"
            )
        self.curPath = {
            "id": row[0],
            "userID": row[1],
            "path": json.loads(row[2])
        }
        self.targetIndex = 0
        self.navState = "start"
        self.navPause = True
        return TriggerWithInfoResponse(
            success=True,
            message=self.curPath["id"]
        )

    def ServiceStopNav(self,request):
        self.navState = "stop"
        self.curPath = None
        return TriggerResponse(
            success=True,
            message=""
        )

    def ServiceSetPause(self,request):
        self.navPause = request.data
        if not self.navPause and self.curPath is not None:
            target = self.curPath["path"]["ptArr"][self.targetIndex]
            print("continue go to target %d, lat: %f, lng: %f" % (self.targetIndex,target["lat"],target["lng"]))

        return SetBoolResponse(
            success=True,
            message=""
        )

    def ServiceSetLoop(self,request):
        self.navLoop = request.data
        return SetBoolResponse(
            success=True,
            message=""
        )

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
            newPt["dLng"] *= math.cos(lastPt["lat"]*math.pi/180.0)
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
            cosLat = math.cos(pt["lat"]*math.pi/180.0)
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
        lng = self.Trans["scale"]*(cosAngle*x-sinAngle*y)/math.cos(lat*math.pi/180.0)+self.Trans["offsetX"]
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
        offsetX = self.fakeInitLng
        offsetY = self.fakeInitLat
        angle = self.fakeInitAngle*math.pi/180.0
        scale = 180.0/(6378137*math.pi)
        #print([scale,angle,offsetX,offsetY])

        cosAngle = math.cos(angle)
        sinAngle = math.sin(angle)
        self.lat = scale*(sinAngle*x+cosAngle*y)+offsetY
        self.lng = scale*(cosAngle*x-sinAngle*y)/math.cos(self.lat*math.pi/180.0)+offsetX
        #print([x,y,self.lat,self.lng])
    def CheckGPSValid(self,lat,lng):
        lat = float(lat)
        lng = float(lng)
        if math.isnan(lat) or math.isnan(lng):
            return False
        if lat < -90 or lat > 90 or lng < -180 or lng > 180:
            return False
        return True

    def AutoDrive(self, curPose):
        if self.navState != "start":
            return
        if self.navPause:
            return
        if self.curPath is None:
            return
        if self.targetIndex < 0 or self.targetIndex >= len(self.curPath["path"]["ptArr"]):
            return
        
        distTolerance = 0.5
        turnScale = 1.0/math.pi
        forwardScale = 0.3
        target = self.curPath["path"]["ptArr"][self.targetIndex]
        targetXY = self.LatLngToXY(target["lat"],target["lng"])
        diffXY = {
            "x": targetXY["x"]-curPose.translation.x,
            "y": targetXY["y"]-curPose.translation.y
        }
        #check if arrive target
        dist = math.sqrt(diffXY["x"]*diffXY["x"]+diffXY["y"]*diffXY["y"])
        if dist < distTolerance:
            self.targetIndex+=1
            if self.targetIndex >= len(self.curPath["path"]["ptArr"]):
                if self.navLoop:
                    self.targetIndex = 0
                else:
                    self.navState = "stop"
                    self.curPath = None
                    print("finish navigation")
                    return

            target = self.curPath["path"]["ptArr"][self.targetIndex]
            print("go to target %d, lat: %f, lng: %f" % (self.targetIndex,target["lat"],target["lng"]))

        #compute drive dirrection
        q = [curPose.rotation.x,curPose.rotation.y,curPose.rotation.z,curPose.rotation.w]
        curAngle = tf.transformations.euler_from_quaternion(q)[2]
        targetAngle = math.atan2(diffXY["y"],diffXY["x"])
        self.drive["targetTurn"] = (targetAngle-curAngle)*turnScale
        self.drive["targetForward"] = dist*forwardScale

        #依超音波測距調整command
        if self.drive["adjustByUS"]:
            turnForce = 0
            turnForceScale = 1
            breakForce = 0
            breakForceScale = 1
            activeDist = 1000
            if self.drive["targetForward"] > 0:
                if self.usDist["FL"] > 0 and self.usDist["FL"] < activeDist:
                    turnForce += turnForceScale/self.usDist["FL"]
                if self.usDist["FR"] > 0 and self.usDist["FR"] < activeDist:
                    turnForce -= turnForceScale/self.usDist["FR"]
                if self.usDist["F"] > 0 and self.usDist["F"] < activeDist:
                    breakForce -= breakForceScale/self.usDist["F"]
            else:
                if self.usDist["BL"] > 0 and self.usDist["BL"] < activeDist:
                    turnForce -= turnForceScale/self.usDist["BL"]
                if self.usDist["BR"] > 0 and self.usDist["BR"] < activeDist:
                    turnForce += turnForceScale/self.usDist["BR"]
                if self.usDist["B"] > 0 and self.usDist["B"] < activeDist:
                    breakForce += breakForceScale/self.usDist["B"]
            self.drive["targetTurn"] += turnForce
            self.drive["targetForward"] += breakForce

        #limit target magnitude
        if self.drive["targetForward"] > 1:
            self.drive["targetForward"] = 1
        elif self.drive["targetForward"] < -1:
            self.drive["targetForward"] = -1
        if self.drive["targetTurn"] > 1:
            self.drive["targetTurn"] = 1
        elif self.drive["targetTurn"] < -1:
            self.drive["targetTurn"] = -1

        #update command by PID controller
        errForward = self.drive["targetForward"] - self.drive["curForward"]
        errTurn = self.drive["targetTurn"] - self.drive["curTurn"]
        errDiffForward = errForward - self.drive["errForward"]
        errDiffTurn = errTurn - self.drive["errTurn"]
        self.drive["errSumForward"] += errForward
        self.drive["errSumTurn"] += errTurn

        self.drive["curForward"] += errForward*self.drive["forwardP"] + self.drive["errSumForward"]*self.drive["forwardI"] + errDiffForward*self.drive["forwardD"]
        
        self.drive["curTurn"] += errTurn*self.drive["turnP"] + self.drive["errSumTurn"]*self.drive["turnI"] + errDiffTurn*self.drive["turnD"]

        self.drive["errForward"] = errForward
        self.drive["errTurn"] = errTurn

        #limit command
        if self.drive["curForward"] > 1:
            self.drive["curForward"] = 1
        elif self.drive["curForward"] < -1:
            self.drive["curForward"] = -1
        if self.drive["curTurn"] > 1:
            self.drive["curTurn"] = 1
        elif self.drive["curTurn"] < -1:
            self.drive["curTurn"] = -1

        #publish command message
        msg = Twist()
        msg.linear.x = self.drive["curForward"]
        msg.angular.z = self.drive["curTurn"]
        self.pubCarCmd.publish(msg)



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
                if self.useFakeGPS:
                    self.GenFakeLatLng(pose.translation.x,pose.translation.y)
                if not self.CheckGPSValid(self.lat,self.lng):
                    print("invalid gps")
                    continue
                if not self.Trans["inited"]:
                    self.Trans["angle"] = 0
                    self.Trans["offsetX"] = self.lng
                    self.Trans["offsetY"] = self.lat
                    self.Trans["inited"] = True
                self.UpdateOdomToGPSTransform(pose.translation.x,pose.translation.y,self.lat,self.lng)
                
                self.AutoDrive(pose)

                #publish navigation state
                latlng = self.XYToLatLng(pose.translation.x,pose.translation.y)
                q = [pose.rotation.x,pose.rotation.y,pose.rotation.z,pose.rotation.w]
                angle = tf.transformations.euler_from_quaternion(q)
                navStateMsg = NavState()
                navStateMsg.lat = latlng["lat"]
                navStateMsg.lng = latlng["lng"]
                navStateMsg.angle = (angle[2]+self.Trans["angle"])*180/math.pi
                navStateMsg.state = self.navState
                navStateMsg.loop = self.navLoop
                navStateMsg.pause = self.navPause
                if self.curPath is not None:
                    navStateMsg.pathID = self.curPath["id"]
                    navStateMsg.targetIndex = self.targetIndex
                else:
                    navStateMsg.pathID = ""
                    navStateMsg.targetIndex = 0
                self.pubNavState.publish(navStateMsg)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                #rospy.logwarn("auto_navigation lookup transform error")
                pass
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('auto_navigation_node')
    rospy.loginfo("auto_navigation_node started")
    an = AutoNavigation()
    an.Run()
