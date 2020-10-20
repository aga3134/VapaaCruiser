#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import yaml
import os

class FSM():
    def __init__(self):
        #use latch=True to send latest state to new subscriber
        self.statePub = rospy.Publisher('fsm/state',String, queue_size=1,latch=True) 

        configFile = rospy.get_param("~configFile","config/fsm.yml")
        if os.path.isfile(configFile):
            with open(configFile) as f:
                self.config = yaml.load(f)
                self.state = self.config["initial_state"]
                self.statePub.publish(String(self.state))
        else:
            rospy.logerr("config file %s not found. abort." % configFile)
            exit()

        self.eventSub = rospy.Subscriber('fsm/event', String, self.DoTransition)

    def DoTransition(self,msg):
        if self.state not in self.config["states"]:
            return rospy.logerr("current state not in state dict.")

        trans = self.config["states"][self.state]["transitions"]
        if not trans:
            return rospy.logerr("No transition in current state")
        
        if msg.data in trans:
            self.state = trans[msg.data]
            rospy.loginfo("recieve event %s, change to state %s" % (msg.data,self.state))
            self.statePub.publish(String(self.state))
        else:
            #check global transition
            if msg.data in self.config["global_transitions"]:
                self.state = self.config["global_transitions"][msg.data]
                rospy.loginfo("recieve global event %s, change to state %s" % (msg.data,self.state))
                self.statePub.publish(String(self.state))
            else:
                rospy.logerr("Invalid transition event %s" % (msg.data))
       

if __name__ == '__main__':
    rospy.init_node('fsm_node')
    rospy.loginfo("fsm_node started")

    fsm = FSM()
    rospy.spin()