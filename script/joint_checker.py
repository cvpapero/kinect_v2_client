#coding: utf-8


"""
2016/9/3
Visualize Positions (x, y, z) from Kinect V2 Server
and
Visualize Speaker
"""

import numpy as np
import json
import os
import copy
import sys
  

import rospy
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from humans_msgs.msg import Humans


class Prediction():
    def __init__(self):

        # set extra data param
        self.sidx = [0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 20] # select joints index
        # color set
        self.carray = []
        clist = [[1, 0, 0, 1], [0, 1, 0, 1], [1, 1, 0, 1], [1, 0.5, 0, 1]]
        for c in clist:
            color = ColorRGBA()
            color.r, color.g, color.b, color.a = c[0], c[1], c[2], c[3]
            self.carray.append(color)
        # set line param
        self.llist = [[3,2,20,1,0],[20,4,5,6,7,21],[6,22],[20,8,9,10,11,23],[10,24],[0,12,13,14,15],[0,16,17,18,19]]

        # set pub/sub
        self.mpub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        self.ksub = rospy.Subscriber('/humans/kinect_v2', Humans, self.callback)
        
        
    def select_data(self, data, idx):
        sidx = [sid*3+i  for sid in idx for i in range(3)]
        return data[sidx]


    def set_point(self, pos, addx=0, addy=0, addz=0):
        pt = Point()            
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
            if human.body.is_speaked == True:
                color = 1
                
            
            # ---points---
            points = []
            pmsg = self.rviz_obj(u, 'p'+str(u), 7, [0.03, 0.03, 0.03], self.carray[0], 0)
            for p in human.body.joints:
                points.append(self.set_point([p.position.x, p.position.y, p.position.z]))
            pmsg.points = points
            msgs.markers.append(pmsg)

            # ---lines---
            lmsg = self.rviz_obj(u, 'l'+str(u), 5, [0.01, 0.01, 0.01], self.carray[color], 0)  
            for ls in self.llist:
                for l in range(len(ls)-1):
                    for add in range(2):
                        lmsg.points.append(points[ls[l+add]])
                msgs.markers.append(lmsg)
         
        self.mpub.publish(msgs)
        


def main(args):
    pd = Prediction()
    rospy.init_node("joint_checker", anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting Down"
        

if __name__ == "__main__":
    print "start"
    main(sys.argv)
