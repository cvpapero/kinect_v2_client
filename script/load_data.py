# coding:utf-8

import json
import numpy as np
import copy

# load data 2 persons
def load_data_persons(filenames, switch=False, select=True, normalize=True):#filename is list
    def json_load_data(filenames):
        stack1, stack2 = [], []
        for filename in filenames:
            print filename         
            for i in range(2): #user num
                f = open(filename, 'r')
                jD = json.load(f)[i]
                f.close()
                data = []
                ds = len(jD["datas"])
                dd = len(jD["datas"][0]["jdata"]) 
                dp = len(jD["datas"][0]["jdata"][0]) #3
                pose = [] #pose: shape(5000, 75)
                for user in jD["datas"]:
                    ps = [] #ps: shape(75,)
                    for ds in user["jdata"]:
                        for p in ds:
                            ps.append(p) #p: shape(3,)
                    pose.append(ps)
                if i == 0:
                    stack1.extend(pose) #user 1
                else:
                    stack2.extend(pose) #user 2
        st1, st2 = copy.deepcopy(stack1), copy.deepcopy(stack2)
            
        if switch == True:
            st1.extend(stack2)
            st2.extend(stack1)
                
        return np.array([st1, st2], dtype=np.float32)
        
    def select_data(data, idx):
        sidx = [sid*3+i  for sid in idx for i in range(3)]
        return data[:, sidx]

    data = json_load_data(filenames) # 2 persons data

    if select==True:
        sidx = [0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 20, 25] # select joints index
    else:
        sidx = np.arange(26)
    
    u1 = select_data(data[0], sidx)
    u2 = select_data(data[1], sidx)

    if normalize==False:
        return np.hstack((u1, u2)), np.hstack((u1, u2))
    
    # std
    su1, su2 = [], []
    for d1, d2 in zip(u1, u2):
        su1.append(normalize_data(d1, 3)[0])
        su2.append(normalize_data(d2, 3)[0])
    su1, su2 = np.array(su1), np.array(su2) 
    
    return np.hstack((su1, su2)), np.hstack((su1, su2))



# data.shape (N,)
def normalize_data(data, os):
    # 原点の計算
    def calc_org(data, s_l_id, s_r_id, spn_id, os):
        #print data
        s_l, s_r, spn = data[s_l_id], data[s_r_id], data[spn_id] 
        a, b = 0, 0
        # 原点 = 右肩から左肩にかけた辺と胸からの垂線の交点
        for i in range(os):
            a += (spn[i]-s_l[i])*(s_r[i]-s_l[i])
            b += (s_r[i]-s_l[i])**2
        k = a/b
        return np.array([k*s_r[i]+(1-k)*s_l[i] for i in range(os)])

    # 法線の計算(from spn)
    def calc_normal(d_sl, d_sp, d_sr):
        l_s = np.array(d_sl) - np.array(d_sp)
        s_r = np.array(d_sp) - np.array(d_sr)
        x = l_s[1]*s_r[2]-l_s[2]*s_r[1]
        y = l_s[2]*s_r[0]-l_s[0]*s_r[2]
        z = l_s[0]*s_r[1]-l_s[1]*s_r[0]
        return np.array([x, y, z])

    # 回転行列による変換
    def calc_rot_pose(data, th_z, th_y, org):
        cos_th_z, sin_th_z = np.cos(-th_z), np.sin(-th_z) 
        cos_th_y, sin_th_y = np.cos(th_y), np.sin(th_y)
        rot1 = np.array([[cos_th_z, -sin_th_z, 0],[sin_th_z, cos_th_z, 0],[0, 0, 1]])
        rot2 = np.array([[cos_th_y, 0, sin_th_y],[0, 1, 0],[-sin_th_y, 0, cos_th_y]])
        rot_pose = []
        for dt in data:
            dt = np.array(dt)-org
            rd = np.dot(rot1, dt)
            rd = np.dot(rot2, rd)
            rd = rd+org
            rot_pose.append(rd)
        return rot_pose

    # 平行移動
    def calc_shift_pose(data, org, s):
        #print org.shape
        shift = s - org
        shift_pose = []
        for dt in data:
            dt += shift
            shift_pose.append(dt)
        return np.array(shift_pose)

    def data_set(data, os):
        ds = np.reshape(data, (len(data)/os, os))
        return ds

    def data_reset(data):
        data = np.array(data, dtype=np.float32) # dtype is important!
        ds = np.reshape(data, (data.shape[0]*data.shape[1], ))
        return ds
    
    #[1,2,3,...] -> [[1,2,3],[...],...]
    data = data_set(data, os)
    
    # 左肩 4, 右肩 7(raw joints was cutted), 胸 1 
    s_l_id, s_r_id, spn_id = 4, 7, 1
    
    # 原点の計算
    org = calc_org(data, s_l_id, s_r_id, spn_id, os)
    # 法線を求める
    normal = calc_normal(data[spn_id], org, data[s_l_id])
    #normal = calc_normal(data[s_l_id], data[spn_id], data[s_r_id])
    
    # 法線の角度方向にposeを回転
    th_z = np.arctan2(normal[1], normal[0])-np.arctan2(org[1], org[0]) #z軸回転(法線と原点の間の角度)
    th_y = 0 #np.arctan2(normal[2], normal[0])-np.arctan2(org[2], org[0]) #y軸回転 
    rot_pose = calc_rot_pose(data, th_z, th_y, org)
    #orgをx軸上に変換する
    th_z = np.arctan2(org[1], org[0])
    th_y = np.arctan2(org[2], org[0])
    rot_pose_norm = calc_rot_pose(rot_pose, th_z, th_y, np.array([0,0,0]))
    # 変換後のorg
    rot_org = calc_org(rot_pose_norm, s_l_id, s_r_id, spn_id, os)
    # orgのxを特定の値に移動する(origen)
    s = [0,0,0]
    shift_pose = calc_shift_pose(rot_pose_norm, rot_org, s)
    shift_org = calc_org(shift_pose, s_l_id, s_r_id, spn_id, os)
    
    #print "reset", data_reset(shift_pose).shape
    return data_reset(shift_pose), shift_org
