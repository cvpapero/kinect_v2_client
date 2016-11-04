#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
2016/10/10
発話と口の動きで喋っているかどうか判定

2016/9/3
Visualize Positions (x, y, z) from Kinect V2 Server
and
Visualize Speaker
"""


import argparse
import numpy as np
import json
import os
import copy
import sys

#PKG = 'joint_checker'
#import roslib; roslib.load_manifest(PKG)

import rospy
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from humans_msgs.msg import Humans

"""
parser = argparse.ArgumentParser()
parser.add_argument('--topic', '-t', default="/humans/kinect_v2",
                    help='topic name')
parser.add_argument('--name', '-n', default="joint_checker",
                    help='node name')
parser.add_argument('--rotate', '-r', default=False, type=bool,
                    help='rotation default False')
arg_ps = parser.parse_args()
"""


class JointChecker():
    def __init__(self, topic1, topic2, rotate1, rotate2):

        self.topics = [topic1, topic2]
        self.rotates = [rotate1, rotate2]
        print "topic:", self.topics
        print "rotate:", self.rotates
        
        self.carray = []
        clist = [[1, 0, 0, 1], [0, 1, 0, 1], [1, 1, 0, 1], [1, 0.5, 0, 1]]
        for c in clist:
            color = ColorRGBA()
            color.r, color.g, color.b, color.a = c[0], c[1], c[2], c[3]
            self.carray.append(color)
        # set line param
        self.llist = [[25,3,2,20,1,0],[20,4,5,6,7,21],[6,22],[20,8,9,10,11,23],[10,24],[0,12,13,14,15],[0,16,17,18,19]]

        # set pub/sub
        self.mpub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=1)

        self.human_1 = Humans()
        self.human_2 = Humans()        

    def start_subscribe(self):
        rospy.Subscriber(self.topics[0], Humans, self.callback_1)
        rospy.Subscriber(self.topics[1], Humans, self.callback_2)

    def callback_1(self, msg):
        self.human_1 = msg
        #print "h1:", self.human_1
        
    def callback_2(self, msg):
        self.human_2 = msg
        #print "h2:", self.human_2
    
    def set_point(self, pos, addx=0, addy=0, addz=0, rotate=False):
        pt = Point()
        if rotate == True:
            pt.x, pt.y, pt.z = -1*pos[0]+1+addx, pos[1]+addy, pos[2]+addz
        else:
            pt.x, pt.y, pt.z = pos[0]+addx, pos[1]+addy, pos[2]+addz
        return pt

    def rviz_obj(self, obj_id, obj_ns, obj_type, obj_size, obj_color=[0, 0, 0, 0], obj_life=0):
        obj = Marker()
        obj.header.frame_id, obj.header.stamp = "camera_link", rospy.Time.now()
        obj.ns, obj.action, obj.type = str(obj_ns), 0, obj_type
        obj.scale.x, obj.scale.y, obj.scale.z = obj_size[0], obj_size[1], obj_size[2]
        obj.color = obj_color
        obj.lifetime = rospy.Duration.from_sec(obj_life)
        obj.pose.orientation.w = 1.0
        return obj


    def viz_publish(self):

        if len(self.human_1.human)==0 or len(self.human_2.human)==0:
            print "pass"         
        else:
            
            humans = [self.human_1.human[0], self.human_2.human[0]]
        
            rospy.loginfo("now recog human:%s",str(len(humans)))

            msgs = MarkerArray()
            for u, human in enumerate(humans):

                color = 2
                if human.body.is_speaked: #and human.body.face_info.properties.mouth_moved=="yes":
                    color = 1
                
            
                # ---points---
                points = []
                pmsg = self.rviz_obj(u, 'p'+str(u), 7, [0.03, 0.03, 0.03], self.carray[0], 1.0)
                for p in human.body.joints:
                    points.append(self.set_point([p.position.x, p.position.y, p.position.z], rotate=self.rotates[u]))
                pmsg.points = points
                msgs.markers.append(pmsg)

            
                # ---lines---
                lmsg = self.rviz_obj(u, 'l'+str(u), 5, [0.01, 0.01, 0.01], self.carray[color], 1.0)  
                for ls in self.llist:
                    for l in range(len(ls)-1):
                        for add in range(2):
                            lmsg.points.append(points[ls[l+add]])
                msgs.markers.append(lmsg)

                # ---text---
                tmsg = self.rviz_obj(u, 't'+str(u), 9, [0, 0, 0.1], self.carray[color], 1.0)
                #points.append()
                head = human.body.joints[3]
                tmsg.pose.position = self.set_point([head.position.x, head.position.y, head.position.z+0.5], rotate=self.rotates[u])
                tmsg.pose.orientation.w = 1.0
                face_prop = human.body.face_info.properties
                tmsg.text = "Happy:"+face_prop.happy+"\n"\
                            +"Engaged:"+face_prop.engaged+"\n"\
                            +"W_glass:"+face_prop.wearing_glasses+"\n"\
                            +"R_eye:"+face_prop.right_eye_closed+"\n"\
                            +"L_eye:"+face_prop.left_eye_closed+"\n"\
                            +"MouthOpen:"+face_prop.mouth_open+"\n"\
                            +"MouthMoved:"+face_prop.mouth_moved+"\n"\
                            +"LookingAway:"+face_prop.looking_away
                msgs.markers.append(tmsg)
            
            self.mpub.publish(msgs)
            


def main(args):

    rospy.init_node("joint_checker", anonymous=True)

    t_1 = rospy.get_param("~topic1", "/humans/kinect_v2/1")
    t_2 = rospy.get_param("~topic2", "/humans/kinect_v2/2")
    r_1 = rospy.get_param("~rotate1", False)
    r_2 = rospy.get_param("~rotate2", True)
    
    jc = JointChecker(t_1, t_2, r_1, r_2)
    
    while not rospy.is_shutdown():
        jc.start_subscribe() 
        jc.viz_publish()
    
    """
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting Down"
    """ 

if __name__ == "__main__":
    print "start"
    main(sys.argv)
