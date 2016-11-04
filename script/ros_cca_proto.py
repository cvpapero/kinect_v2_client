#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
2016.1.17
計算するだけ
結果の可視化は別のモジュールにバトンタッチ
可視化はしなくてよい

2016.1.11
決定版をつくる

"""

import sys
import os.path
import math
import json
import time
from datetime import datetime

import numpy as np
from numpy import linalg as NLA
import scipy as sp
from scipy import linalg as SLA

from PyQt4 import QtGui, QtCore
from PyQt4.QtCore import *
from PyQt4.QtGui  import *

#import pylab as pl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as pl
import matplotlib.animation as animation

import rospy

#import pyper
import pandas as pd

import rospy
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from std_msgs.msg import ColorRGBA


class CCA(QtGui.QWidget):

    def __init__(self):
        super(CCA, self).__init__()
        #UIの初期化
        self.initUI()

        #ROSのパブリッシャなどの初期化
        rospy.init_node('roscca', anonymous=True)
        self.mpub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        self.ppub = rospy.Publisher('joint_diff', PointStamped, queue_size=10)

        #rvizのカラー設定(未)
        self.carray = []
        clist = [[1,0,0,1],[0,1,0,1],[1,1,0,1]]
        for c in clist:
            color = ColorRGBA()
            color.r = c[0]
            color.g = c[1]
            color.b = c[2]
            color.a = c[3]
            self.carray.append(color) 

    def initUI(self):
        grid = QtGui.QGridLayout()
        form = QtGui.QFormLayout()
        
        #ファイル入力ボックス
        self.txtSepFile = QtGui.QLineEdit()
        btnSepFile = QtGui.QPushButton('...')
        btnSepFile.setMaximumWidth(40)
        btnSepFile.clicked.connect(self.chooseDbFile)
        boxSepFile = QtGui.QHBoxLayout()
        boxSepFile.addWidget(self.txtSepFile)
        boxSepFile.addWidget(btnSepFile)
        form.addRow('input file', boxSepFile)
    
        #ファイル出力
        self.txtSepFileOut = QtGui.QLineEdit()
        btnSepFileOut = QtGui.QPushButton('...')
        btnSepFileOut.setMaximumWidth(40)
        btnSepFileOut.clicked.connect(self.chooseOutFile)
        boxSepFileOut = QtGui.QHBoxLayout()
        boxSepFileOut.addWidget(self.txtSepFileOut)
        boxSepFileOut.addWidget(btnSepFileOut)
        form.addRow('output file', boxSepFileOut)    

        #window size
        self.winSizeBox = QtGui.QLineEdit()
        self.winSizeBox.setText('90')
        self.winSizeBox.setAlignment(QtCore.Qt.AlignRight)
        self.winSizeBox.setFixedWidth(100)
        form.addRow('window size', self.winSizeBox)

        #frame size
        self.frmSizeBox = QtGui.QLineEdit()
        self.frmSizeBox.setText('110')
        self.frmSizeBox.setAlignment(QtCore.Qt.AlignRight)
        self.frmSizeBox.setFixedWidth(100)
        form.addRow('frame size', self.frmSizeBox)

        #selected joints
        self.selected = QtGui.QRadioButton('selected')
        form.addRow('dimension', self.selected)

        #output file
        boxFile = QtGui.QHBoxLayout()
        btnOutput = QtGui.QPushButton('output')
        btnOutput.clicked.connect(self.doOutput)
        form.addWidget(btnOutput)

        #exec
        boxCtrl = QtGui.QHBoxLayout()
        btnExec = QtGui.QPushButton('exec')
        btnExec.clicked.connect(self.doExec)
        #btnExec.clicked.connect(self.manyFileExec)
        boxCtrl.addWidget(btnExec)

        #テーブルの初期化
        #horizonはuser2の時間
        self.table = QtGui.QTableWidget(self)
        self.table.setColumnCount(0)
        self.table.setHorizontalHeaderLabels("use_2 time") 
        jItem = QtGui.QTableWidgetItem(str(0))
        self.table.setHorizontalHeaderItem(0, jItem)

        #アイテムがクリックされたらグラフを更新
        self.table.itemClicked.connect(self.updateColorTable)
        self.table.setItem(0, 0, QtGui.QTableWidgetItem(1))

        boxTable = QtGui.QHBoxLayout()
        boxTable.addWidget(self.table)
 
        #配置
        grid.addLayout(form,1,0)
        grid.addLayout(boxCtrl,2,0)
        #grid.addLayout(boxUpDown,3,0)
        grid.addLayout(boxTable,4,0)

        self.setLayout(grid)
        self.resize(400,100)

        self.setWindowTitle("cca window")
        self.show()

    def chooseDbFile(self):
        dialog = QtGui.QFileDialog()
        dialog.setFileMode(QtGui.QFileDialog.ExistingFile)
        if dialog.exec_():
            fileNames = dialog.selectedFiles()
            for f in fileNames:
                self.txtSepFile.setText(f)
                return
        return self.txtSepFile.setText('')

    def chooseOutFile(self):
        dialog = QtGui.QFileDialog()
        dialog.setFileMode(QtGui.QFileDialog.ExistingFile)
        if dialog.exec_():
            fileNames = dialog.selectedFiles()
            for f in fileNames:
                self.txtSepFileOut.setText(f)
                return
        return self.txtSepFileOut.setText('')

    def updateColorTable(self, cItem):
        self.r = cItem.row()
        self.c = cItem.column()
        print "now viz r:",self.r,", c:",self.c
        #print "cca:",self.r_m[r][c]

        #pj1 = self.wx_m[self.r,self.c,:,0].argmax()
        #pj2 = self.wy_m[self.r,self.c,:,0].argmax()
        th = 0.7
        p1 = []
        p2 = []
        for idx,v in enumerate(self.wx_m[self.r,self.c,:,0]):
            if v > th:
                iv=[]
                iv.append(idx)
                iv.append(v)
                p1.append(iv)
        for idx,v in enumerate(self.wx_m[self.r,self.c,:,0]):
            if v > th:
                iv=[]
                iv.append(idx)
                iv.append(v)
                p2.append(iv)

        if len(p1) == 0:
            iv=[]
            iv.append(self.wx_m[self.r,self.c,:,0].argmax())
            iv.append(self.wx_m[self.r,self.c,:,0].max())
            p1.append(iv)
        if len(p2) == 0:
            iv=[]
            iv.append(self.wy_m[self.r,self.c,:,0].argmax())
            iv.append(self.wy_m[self.r,self.c,:,0].max())
            p2.append(iv)

        self.doPub(self.r, self.c, self.pos1, self.pos2, self.wins, p1, p2)
        #self.doViz()
        #self.doViz(self.r, self.c, self.pos1, self.pos2, self.wins, pj1, pj2)
        #GRAPH().drawPlot(r, c, self.wx_m, self.wy_m, self.r_m, self.data1, self.data2, self.dtmr, self.dtd, self.wins)
        #GRAPH().vecPlot(r, c, self.r_m, self.wx_m, self.wy_m, self.data1, self.data2, self.frms, self.wins)
        #GRAPH().vecPlot2(r, c, self.r_m, self.wx_m, self.wy_m, self.data1, self.data2, self.dtmr, self.frms, self.wins)
        
    def updateTable(self):
        #GRAPH().rhoPlot(self.r_m, self.filename, self.wins, self.frms)
        th = 0#float(self.ThesholdBox.text())
        if(len(self.r_m)==0):
            print "No Corr Data! Push exec button..."
        self.table.clear()
        font = QtGui.QFont()
        font.setFamily(u"DejaVu Sans")
        font.setPointSize(5)
        self.table.horizontalHeader().setFont(font)
        self.table.verticalHeader().setFont(font)
        self.table.setRowCount(len(self.r_m))
        self.table.setColumnCount(len(self.r_m))
        for i in range(len(self.r_m)):
            jItem = QtGui.QTableWidgetItem(str(i))
            self.table.setHorizontalHeaderItem(i, jItem)
        hor = True
        for i in range(len(self.r_m)):
            iItem = QtGui.QTableWidgetItem(str(i))
            self.table.setVerticalHeaderItem(i, iItem)
            self.table.verticalHeaderItem(i).setToolTip(str(i))
            #時間軸にデータを入れるなら↓
            #self.table.verticalHeaderItem(i).setToolTip(str(self.timedata[i]))
            for j in range(len(self.r_m[i])):
                if hor == True:
                    jItem = QtGui.QTableWidgetItem(str(j))
                    self.table.setHorizontalHeaderItem(j, jItem)
                    self.table.horizontalHeaderItem(j).setToolTip(str(j))
                    hot = False
                c = 0
                rho = round(self.r_m[i][j][0],5)
                rho_data = str(round(self.r_m[i][j][0],5))+", "+str(round(self.r_m[i][j][1],5))+", "+str(round(self.r_m[i][j][2],5))
                if rho > th:
                    c = rho*255
                self.table.setItem(i, j, QtGui.QTableWidgetItem())
                self.table.item(i, j).setBackground(QtGui.QColor(c,c,c))
                self.table.item(i, j).setToolTip(str(rho_data))
        self.table.setVisible(False)
        self.table.resizeRowsToContents()
        self.table.resizeColumnsToContents()
        self.table.setVisible(True)



    def manyFileExec(self):
        print "many file exec"
        #fnames=["20151222_a1.json","20151222_a2.json","20151222_a3.json","20151222_b1.json","20151222_b2.json","20151222_b3.json"]
        fnames=["20151222_a1.json","20151222_a2.json"]


        for fn in fnames: 
            print "---now file", fn, "---"
            self.data1,self.data2,self.pos1,self.pos2,self.time = self.jsonInput(fn)
            hitbtn = self.selected.isChecked()
            if hitbtn == True: 
                self.data1, self.data2 = self.selectInput(self.data1, self.data2)
                #if data is big then...
                #self.data1, self.data2 = self.cutDatas(self.data1, self.data2, 300)

            self.data1, self.data2 = self.cutDatas(self.data1, self.data2, 300)
            self.wins = int(self.winSizeBox.text())
            self.frms = int(self.frmSizeBox.text())
            #self.frms = self.dts

            #dmr:data_max_range, frmr:frame_range, dtr:data_range
            self.dtmr = self.dts - self.wins + 1
            self.frmr = self.dts - self.frms + 1
            self.dtr = self.frms - self.wins + 1

            print "datas_size:",self.dts
            print "data_max_range:",self.dtmr
            print "frame_range:",self.frmr
            print "data_range:",self.dtr

            #rho_m:rho_matrix[dmr, dmr, datadimen] is corrs
            #wx_m and wy_m is vectors
            self.r_m, self.wx_m, self.wy_m = self.ccaExec(self.data1, self.data2)
            self.fname = fn
            self.doOutput()
            print "---end---"

    def doExec(self):
        #print "exec!"

        self.data1 = []
        self.data2 = []

        #ws:window_size, fs:frame_size 
        self.wins = int(self.winSizeBox.text())
        self.frms = int(self.frmSizeBox.text())

        hitbtn = self.selected.isChecked()
        #print "setect joints:"+str(hitbtn)
        #input file
        self.fname = str(self.txtSepFile.text())
        self.data1,self.data2,self.pos1,self.pos2,self.time = self.jsonInput(self.fname)
        #select joints
        if hitbtn == True: 
            self.data1, self.data2 = self.selectInput(self.data1, self.data2)
        #if data is big then...
        self.data1, self.data2 = self.cutDatas(self.data1, self.data2, 300)

        #dmr:data_max_range, frmr:frame_range, dtr:data_range
        self.dtmr = self.dts - self.wins + 1
        self.frmr = self.dts - self.frms + 1
        self.dtr = self.frms - self.wins + 1

        #print "datas_size:",self.dts
        #print "data_max_range:",self.dtmr
        #print "frame_range:",self.frmr
        #print "data_range:",self.dtr

        #rho_m:rho_matrix[dmr, dmr, datadimen] is corrs
        #wx_m and wy_m is vectors
        self.r_m, self.wx_m, self.wy_m = self.ccaExec(self.data1, self.data2)
        #self.updateTable()
        print "end"

    def jsonInput(self, filename):
        f = open(filename, 'r')
        jsonData = json.load(f)
        f.close()
        #angle
        datas = []
        for user in jsonData:
            #data is joint angle
            data = []
            #dts:data_size, dtd:data_dimension
            self.dts = len(user["datas"])
            self.dtd = len(user["datas"][0]["data"])
            for j in range(self.dts):
                data.append(user["datas"][j]["data"])
            datas.append(data)

        
        poses = []
        for user in jsonData:
            pos = []
            psize = len(user["datas"][0]["jdata"])
            for j in range(self.dts):
                pls = []
                for p in range(psize):
                    pl = []
                    for xyz in range(3):
                        pl.append(user["datas"][j]["jdata"][p][xyz])
                    pls.append(pl)
                pos.append(pls)
            poses.append(pos)

        #time ただし1.14現在,値が入ってない
        time = []
        for t in jsonData[0]["datas"]:
            time.append(t["time"])
        
        #print "poses[0]:",poses[0]

        #可視化用,ジョイント
        f = open('/home/uema/catkin_ws/src/rqt_cca/joint_index.json', 'r')
        jsonIdxDt = json.load(f)
        f.close
        self.jIdx = []
        for idx in jsonIdxDt:
            jl = []
            for i in idx:
                jl.append(i)
            self.jIdx.append(jl)
        
        return datas[0], datas[1], poses[0], poses[1], time

    #def jsonPosInput(self, filename):
    def doOutput(self):
        print "save start:",datetime.now().strftime("%Y/%m/%d %H:%M:%S")
        savefile = "save_" + self.fname.lstrip("/home/uema/catkin_ws/src/rqt_cca/data2/") 
        print "filename:",savefile
        f = open(savefile ,'w')
        prop = {"wins":self.wins, "frms":self.frms, "dtd":self.dtd, "dts":self.dts, "fname":self.fname}
        cca = {"r":self.r_m[:,:,0].tolist(),"wx":self.wx_m[:,:,:,0].tolist(), "wy":self.wy_m[:,:,:,0].tolist()}
        #cca = {"r":self.r_m.tolist(),"wx":self.wx_m.tolist(), "wy":self.wy_m.tolist()}
        js = {"prop":prop, "cca":cca}
        jsons = json.dumps(js)
        f.write(jsons)
        f.close()
        print "save end:",datetime.now().strftime("%Y/%m/%d %H:%M:%S")

    def selectInput(self, data1, data2):
        idx = [3, 4, 5, 6, 11, 12, 23, 24, 25, 26, 27, 28]
        #idx = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,0]
        #idx = [1,2,3,0]
        self.dtd = len(idx)
        datas1 = []
        datas2 = []
        for (ds1, ds2) in zip(data1, data2):
            #print "data:"+str(data)
            use1 = []
            use2 = []
            for i in idx:                   
                use1.append(ds1[i])
                use2.append(ds2[i])
            datas1.append(use1)
            datas2.append(use2)
        return datas1, datas2

    def cutDatas(self, data1, data2, th):        
        if self.dts > th:
            datas1 = []
            datas2 = []
            for i,(ds1,ds2) in enumerate(zip(data1,data2)):
                if i < th:
                    datas1.append(ds1)
                    datas2.append(ds2)
                else:
                    break
            self.dts = len(datas1)
            return datas1, datas2
        else:
            return data1, data2


    def ccaExec(self, data1, data2):
        #rho_m:rho_matrix[dmr, dmr, datadimen] is corrs
        #wx_m and wy_m is vectors

        r_m = np.zeros([self.dtmr, self.dtmr, self.dtd])
        wx_m = np.zeros([self.dtmr, self.dtmr, self.dtd, self.dtd])
        wy_m = np.zeros([self.dtmr, self.dtmr, self.dtd, self.dtd])

        for f in range(self.frmr):
            print "f: ",f,"/",self.frmr-1
            if f == 0:
                for t1 in range(self.dtr):
                    for t2 in range(self.dtr):
                        u1 = []
                        u2 = []
                        for w in range(self.wins):
                            u1.append(data1[t1+f+w])
                            u2.append(data2[t2+f+w])
                        r_m[t1+f][t2+f], wx_m[t1+f][t2+f], wy_m[t1+f][t2+f] = self.cca(u1, u2)
            else:
                od = f+self.dtr-1
                for t1 in range(self.dtr-1):
                    u1 = []
                    u2 = []
                    for w in range(self.wins):
                        u1.append(data1[f+t1+w])
                        u2.append(data2[od+w])
                    r_m[t1+f][od], wx_m[t1+f][od], wy_m[t1+f][od] = self.cca(u1, u2)
                for t2 in range(self.dtr):
                    u1 = []
                    u2 = []
                    for w in range(self.wins):
                        u1.append(data1[od+w])
                        u2.append(data2[f+t2+w])
                    r_m[od][f+t2], wx_m[od][f+t2], wy_m[od][f+t2] = self.cca(u1, u2)
        return r_m, wx_m, wy_m


    def cca(self, X, Y):
        '''
        正準相関分析
        http://en.wikipedia.org/wiki/Canonical_correlation
        '''    
        X = np.array(X)
        Y = np.array(Y)
        n, p = X.shape
        n, q = Y.shape
        
        # zero mean
        X = X - X.mean(axis=0)
        Y = Y - Y.mean(axis=0)
        
        # covariances
        S = np.cov(X.T, Y.T, bias=1)
        
        # S = np.corrcoef(X.T, Y.T)
        SXX = S[:p,:p]
        SYY = S[p:,p:]
        SXY = S[:p,p:]
        SYX = S[p:,:p]
        
        # 
        sqx = SLA.sqrtm(SLA.inv(SXX)) # SXX^(-1/2)
        sqy = SLA.sqrtm(SLA.inv(SYY)) # SYY^(-1/2)
        M = np.dot(np.dot(sqx, SXY), sqy.T) # SXX^(-1/2) * SXY * SYY^(-T/2)
        A, s, Bh = SLA.svd(M, full_matrices=False)
        B = Bh.T      
        #print np.dot(np.dot(A[:,0].T,SXX),A[:,0])
        return s, A, B

    def doViz(self):
        #self.pos1, self.pos2, self.wins
        pj1 = self.wx_m[self.r,self.c,:,0].argmax()
        pj2 = self.wy_m[self.r,self.c,:,0].argmax()

        pl.ion()
        #self.fig = pl.figure()
        #ax = self.fig.add_subplot(111,projection='3d')
        pl.subplot(111,projection='3d')
        p1 = np.array(self.pos1[self.r])
        pl.scatter(p1[0:self.dtd,0],p1[0:self.dtd,1],p1[0:self.dtd,2],c="r")
        #self.updateViz()
        pl.draw()

    def doPub(self, r, c, pos1, pos2, wins, p1, p2):
        print "---play back start---"
        print p1
        print p2
        if r > c:
            self.pubViz(r, c, pos1[c:r+wins], pos2[c:r+wins], wins, p1, p2)
        else:
            self.pubViz(r, c, pos1[r:c+wins], pos2[r:c+wins], wins, p1, p2)
        print "---play back end---"
        print " "

    #データを可視化するだけでok
    def pubViz(self, r, c, pos1, pos2, wins, p1, p2):

        rate = rospy.Rate(10)
        #print "pos1[0]:",pos1[0]
        poses = []
        poses.append(pos1)
        poses.append(pos2)
        ps = []
        ps.append(p1)
        ps.append(p2)
        for i in range(len(poses[0])):
            print "frame:",i
            if i > r and i < r+wins:
                print "now u1 output"
            if i > c and i < c+wins:
                print "now u2 output"

            msgs = MarkerArray()
            for u, pos in enumerate(poses):
                #points
                pmsg = Marker()
                pmsg.header.frame_id = 'camera_link'
                pmsg.header.stamp = rospy.Time.now()
                pmsg.ns = 'p'+str(u)
                pmsg.action = 0
                pmsg.id = u
                pmsg.type = 7
                js = 0.03
                pmsg.scale.x = js
                pmsg.scale.y = js
                pmsg.scale.z = js
                pmsg.color = self.carray[u]
                for j, p in enumerate(pos[i]):
                    point = Point()
                    point.x = p[0]
                    point.y = p[1]
                    point.z = p[2]
                    pmsg.points.append(point)
                pmsg.pose.orientation.w = 1.0
                msgs.markers.append(pmsg)    

                #note points その時が来たら、表示する(今は最初から上書きしてる)
                npmsg = Marker()
                npmsg.header.frame_id = 'camera_link'
                npmsg.header.stamp = rospy.Time.now()
                npmsg.ns = 'np'+str(u)
                npmsg.action = 0
                npmsg.id = u
                npmsg.type = 7
                njs = 0.05
                npmsg.scale.x = njs
                npmsg.scale.y = njs
                npmsg.scale.z = njs
                npmsg.color = self.carray[2]
                for j in range(len(ps[u])):
                    point = Point()
                    point.x = pos[i][ps[u][j][0]][0]
                    point.y = pos[i][ps[u][j][0]][1]
                    point.z = pos[i][ps[u][j][0]][2]
                    npmsg.points.append(point)
                npmsg.pose.orientation.w = 1.0
                msgs.markers.append(npmsg) 

                #lines
                lmsg = Marker()
                lmsg.header.frame_id = 'camera_link'
                lmsg.header.stamp = rospy.Time.now()
                lmsg.ns = 'l'+str(u)
                lmsg.action = 0
                lmsg.id = u
                lmsg.type = 5
                lmsg.scale.x = 0.005
                lmsg.color = self.carray[2]
                for jid in self.jIdx:
                    for pi in range(2):
                        for add in range(2):
                            point = Point()
                            point.x = pos[i][jid[pi+add]][0]
                            point.y = pos[i][jid[pi+add]][1]
                            point.z = pos[i][jid[pi+add]][2]
                            lmsg.points.append(point) 
                lmsg.pose.orientation.w = 1.0
                msgs.markers.append(lmsg)
            
            self.mpub.publish(msgs)
            rate.sleep()
        

def main():
    app = QtGui.QApplication(sys.argv)
    corr = CCA()
    #graph = GRAPH()
    sys.exit(app.exec_())

if __name__=='__main__':
    main()
