#!/usr/bin/python
# -*- coding: utf-8 -*-


"""
2016.10.19----
interaction dataをannotationするためのinterface

"""


import sys
import os.path
import math
import json
import time

import numpy as np

from PyQt4 import QtGui, QtCore
from PyQt4.QtCore import *
from PyQt4.QtGui  import *

import rospy
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from std_msgs.msg import ColorRGBA

import load_data



class ANNOTATION(QtGui.QWidget):

    def __init__(self):
        super(ANNOTATION, self).__init__()
        #UIの初期化
        self.initUI()

        #ROSのパブリッシャなどの初期化
        rospy.init_node('annotation_interface', anonymous=True)
        self.mpub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        #self.ppub = rospy.Publisher('joint_diff', PointStamped, queue_size=10)

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

        #Botton Objectの作成
        def boxBtnObj(name, func):
            box = QtGui.QHBoxLayout()
            btn = QtGui.QPushButton(name)
            btn.clicked.connect(func)
            box.addWidget(btn)
            return box

        
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

        #input data
        boxInput = boxBtnObj("input", self.inputData)
        #output data
        boxOutput = boxBtnObj("output", self.outputData)
        #stop moving 
        boxStop = boxBtnObj("stop", self.stopData)

        
        """
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

        """



        
        #テーブルの初期化
        #horizonはuser2の時間
        self.table = QtGui.QTableWidget(self)
        self.table.setColumnCount(0)
        self.table.setHorizontalHeaderLabels("use_2 time") 
        jItem = QtGui.QTableWidgetItem(str(0))
        self.table.setHorizontalHeaderItem(0, jItem)

        #アイテムがクリックされたらグラフを更新
        self.table.itemClicked.connect(self.clickUpdateTable)
        self.table.setItem(0, 0, QtGui.QTableWidgetItem(1))

        boxTable = QtGui.QHBoxLayout()
        boxTable.addWidget(self.table)
        
        
        #配置
        grid.addLayout(form,1,0)
        grid.addLayout(boxInput,2,0)
        grid.addLayout(boxOutput,3,0)
        grid.addLayout(boxStop,4,0)
        grid.addLayout(boxTable,5,0)

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


    def updateTable(self, data):

        th = 0#float(self.ThesholdBox.text())
        if(len(data)==0):
            print "No Corr Data! Push exec button..."

        t_row, t_col = data.shape
        
        self.table.clear()
        font = QtGui.QFont()
        font.setFamily(u"DejaVu Sans")
        font.setPointSize(5)
        self.table.horizontalHeader().setFont(font)
        self.table.verticalHeader().setFont(font)

        
        ann_num = 3
        self.table.setColumnCount(t_row)
        self.table.setRowCount(ann_num)
        #self.table.setRowCount(data.shape[0])
        #self.table.setColumnCount(ann_num)
          
        """
        # 軸の値をSet
        for i in range(len(self.r_m)):
            jItem = QtGui.QTableWidgetItem(str(i))
            self.table.setHorizontalHeaderItem(i, jItem)
        """


        
        hor = True
        for i in range(ann_num):
            iItem = QtGui.QTableWidgetItem(str(i))
            self.table.setVerticalHeaderItem(i, iItem)
            self.table.verticalHeaderItem(i).setToolTip(str(i))
            #時間軸にデータを入れるなら↓
            #self.table.verticalHeaderItem(i).setToolTip(str(self.timedata[i]))
            
            for j in range(t_row):
                if hor == True:
                    jItem = QtGui.QTableWidgetItem(str(j))
                    self.table.setHorizontalHeaderItem(j, jItem)
                    self.table.horizontalHeaderItem(j).setToolTip(str(j))
                    hot = False

                color = [255, 255, 255]
                set_data = 0
                
                self.table.setItem(i, j, QtGui.QTableWidgetItem())
                self.table.item(i, j).setBackground(QtGui.QColor(color[0],color[1],color[2]))
                self.table.item(i, j).setToolTip(str(set_data))
                          
        self.table.setVisible(False)
        self.table.resizeRowsToContents()
        self.table.resizeColumnsToContents()
        self.table.setVisible(True)


    # TableがClickされたとき
    def clickUpdateTable(self, cItem):

        self.tip = int(cItem.toolTip())
        self.r = cItem.row()
        self.c = cItem.column()
        print "r:",self.r,", c:",self.c, ". tip:",self.tip
        
        color = [[0, 0, 0], [255, 255, 255]]
        
        set_data = 0 if self.tip == 1 else 1 
                
        self.table.setItem(self.r, self.c, QtGui.QTableWidgetItem())
        self.table.item(self.r, self.c).setBackground(QtGui.QColor(color[self.tip][0],color[self.tip][1],color[self.tip][2]))
        self.table.item(self.r, self.c).setToolTip(str(set_data))


        #jointsの可視化
        # self.vizJoint(self.c)

        
        

    def inputData(self):
        print "input"
        #input file
        self.fname = [str(self.txtSepFile.text())]
        input_data = load_data.load_data_persons(self.fname, switch=False)[0]
        print "data shape:", input_data.shape
        
        self.updateTable(input_data)
        print "end"

        
    def outputData(self):
        print "pass"

        
    def stopData(self):
        print "pass"

        


        
def main():
    app = QtGui.QApplication(sys.argv)
    anotation = ANNOTATION()
    #graph = GRAPH()
    sys.exit(app.exec_())

if __name__=='__main__':
    main()
