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
import math

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
    def __init__(self, topic, rotate):

        # set extra data param
        #self.sidx = [0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 20, 25] # select joints index
        # color set


        self.topic = topic
        self.rotate = rotate
        print "topic:", self.topic
        print "rotate:", self.rotate
        
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
        self.ksub = rospy.Subscriber(self.topic, Humans, self.callback)
        #self.ksub = rospy.Subscriber(arg_ps.topic, Humans, self.callback)
        
    """
    def select_data(self, data, idx):
        sidx = [sid*3+i  for sid in idx for i in range(3)]
        return data[sidx]
    """

    
    def set_point(self, pos, addx=0, addy=0, addz=0, rotate=False):
        pt = Point()
        if rotate == True:
            pt.x, pt.y, pt.z = -1*pos[0]+1+addx, -1*pos[1]+addy, pos[2]+addz
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


    def callback(self, msg):
        
        rospy.loginfo("now recog human:%s",str(len(msg.human)))

        msgs = MarkerArray()
        for u, human in enumerate(msg.human):

            color = 2
            speaked = 1 if human.body.is_speaked > 0.005 else 0 
            #speaked = 1 if 20*math.log10(human.body.is_speaked) > 50 else 0 
            
            if speaked: #and human.body.face_info.properties.mouth_moved=="yes":
                color = 1

            rotate = False
            """
            if self.rotate and (u+1)%2 != 0:
                rotate = True
            """
            # 最初の人(u=0)だけ回転して表示
            if self.rotate and u == 0:
                rotate = True
            # ---points---
            points = []
            pmsg = self.rviz_obj(u, 'p'+str(u), 7, [0.03, 0.03, 0.03], self.carray[0], 1.0)
            for p in human.body.joints:
                points.append(self.set_point([p.position.x, p.position.y, p.position.z], rotate=rotate))
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
            tmsg.pose.position = self.set_point([head.position.x, head.position.y, head.position.z+0.5], rotate=rotate)
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

    #rospy.init_node(arg_ps.name, anonymous=True)
    node_name = "joint_checker" #arg_ps.name
    rospy.init_node(node_name, anonymous=True)

    topic = rospy.get_param("~topic", "/humans/kinect_v2")
    rotate = rospy.get_param("~rotate", False)
    
    jc = JointChecker(topic, rotate)
       
    #rospy.init_node("joint_checker")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting Down"
        

if __name__ == "__main__":
    print "start"
    main(sys.argv)
