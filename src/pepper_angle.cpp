/*
kinectから得たOrientationをPepperの各関節角度に変換


*/


#include "ros/ros.h"
#include "humans_msgs/Humans.h"
#include "humans_msgs/Body.h"
#include "humans_msgs/Joints.h"

#include "geometry_msgs/Quaternion.h"

#include <math.h>


using namespace std;


class Angles
{
public:
  double LShoulderPitch;
  double LShoulderRoll;
  double LElbowYaw;
  double LElbowRoll;
  double RShoulderPitch;
  double RShoulderRoll;
  double RElbowYaw;
  double RElbowRoll;
  double HipPitch;
  double HeadYaw;
};






class PepperAngle
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  ros::Subscriber sub;

  //map< long long, vector<JData> > jpersons;

  //string joint_filename;
  //string output_filename;
  //stringstream out_file;

  geometry_msgs::Quaternion unitX, unitY, unitZ;
  
  double pre_sec;
  int count;
  int arm_idx[2][3] = {{4, 5, 6}, {8, 9, 10}};//腕のid
  
public:
  PepperAngle()
    :pnh("~")
  {
    sub = nh.subscribe("/humans/kinect_v2", 1, &PepperAngle::callback, this);
    //pnh.param<std::string>("output", output_filename, "test");
    //out_file << output_filename << ".json";
    pre_sec = ros::Time::now().toSec();
    count = 0;

    unitX.x = 1; //(1,0,0,0)
    unitY.y = 1;
    unitZ.z = 1;

    //right_arm_idx 
    //left_arm_idx = ;
    
  }


  Angles SetAnglesFromBody(humans_msgs::Body body)
  {
    //右腕の角度を計算
    //right_arm = SetRightArmAngles(body);
    //0:left, 1:right
    Angles angle;
    SetArmAngles(body, 0, &angle);
    SetArmAngles(body, 1, &angle);

    //cout << angle.LShoulderPitch <<","<<angle.LShoulderPitch<< endl;
    //SetHands(body);
    SetHip(body, &angle);
    //SetWristYaws(body);
    SetHead(body, &angle);
    
    //std::cout << robot << std::endl;
    //
    return angle;
  }


  geometry_msgs::Quaternion ProductQ(geometry_msgs::Quaternion v,
				     geometry_msgs::Quaternion u)
  {
    geometry_msgs::Quaternion dist;
    dist.w = v.w * u.w - v.x * u.x - v.y * u.y - v.z * u.z;
    dist.x = v.w * u.x + v.x * u.w + v.y * u.z - v.z * u.y;
    dist.y = v.w * u.y + v.y * u.w + v.z * u.x - v.x * u.z;
    dist.z = v.w * u.z + v.z * u.w + v.x * u.y - v.y * u.x;
    return dist;
  }


  geometry_msgs::Quaternion Cross(geometry_msgs::Quaternion v,
				  geometry_msgs::Quaternion u)
  {
    geometry_msgs::Quaternion dist;
    dist = ProductQ(v, u);
  }

  
  double Product(geometry_msgs::Quaternion a,
		 geometry_msgs::Quaternion b)
  {
    double dist = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
    return dist;
  }


  
  geometry_msgs::Quaternion RotateBy(geometry_msgs::Quaternion unit,
				     geometry_msgs::Quaternion rot)
  {

    //Conjugateは共役という意味  
    geometry_msgs::Quaternion conjRot;
    conjRot.x = -1*rot.x;
    conjRot.y = -1*rot.y;
    conjRot.z = -1*rot.z;
    conjRot.w = rot.w;
    
    //クォータニオンの回転の式
    geometry_msgs::Quaternion dist;
    dist = ProductQ(ProductQ(rot, unit), conjRot);
    return dist;    
  }


  //いまだけvoid
  void SetArmAngles(humans_msgs::Body body, int arm_id, Angles *angle)
  {
    
    // 右肩(ID:8)のクォータニオンを取得
    geometry_msgs::Quaternion shoulder;
    shoulder = body.joints[arm_idx[arm_id][0]].orientation;    
    // 右肩の基底ベクトルを取得する(x, y, z)
    // たとえば、x軸の基底ベクトルなら、
    //unitX=[1, 0, 0]というベクトルを
    //shoulderというクォータニオンで回転させる
    //unitX=[1, 0, 0, 0]; //Quartanion型のため四次元ベクトル
    geometry_msgs::Quaternion shoulderX = RotateBy(unitX, shoulder);
    //unitY=[0, 1, 0, 0]; //Quartanion型のため四次元ベクトル
    geometry_msgs::Quaternion shoulderY = RotateBy(unitY, shoulder);
    //unitZ=[0, 0, 1, 0]; //Quartanion型のため四次元ベクトル
    geometry_msgs::Quaternion shoulderZ = RotateBy(unitZ, shoulder);

    // 右ヒジ(ID:9)のクォータニオンを取得
    geometry_msgs::Quaternion elbowOrientation;
    elbowOrientation = body.joints[arm_idx[arm_id][1]].orientation;
    // ヒジ回転軸ベクトルを取得
    geometry_msgs::Quaternion elbowZ = RotateBy(unitZ, elbowOrientation);
    geometry_msgs::Quaternion elbowY = RotateBy(unitY, elbowOrientation);

    //ここで何んらかのばぐ
    //肩座標を基準にするため成分を再取得
    geometry_msgs::Quaternion elbowZFromShoulder;
    elbowZFromShoulder.x = Product(elbowZ, shoulderX);
    elbowZFromShoulder.y = Product(elbowZ, shoulderY);
    elbowZFromShoulder.z = Product(elbowZ, shoulderZ);
    geometry_msgs::Quaternion elbowPosFromShoulder;
    elbowPosFromShoulder.x = Product(elbowY, shoulderX);
    elbowPosFromShoulder.y = Product(elbowY, shoulderY);
    elbowPosFromShoulder.z = Product(elbowY, shoulderZ);
        
    //ヒジ-手首(ID:10)方向の単位ベクトルを取得
    geometry_msgs::Quaternion wristY;
    wristY = RotateBy(unitY, body.joints[arm_idx[arm_id][2]].orientation);
    
    //材料は揃った。あとは変換計算するだけ  
    //https://github.com/malaybaku/KinectForPepper/blob/master/src/KinectForPepper/Models/BodyToJointAngle.cs
    //肩から見たヒジの位置情報をもとにShoulderRollおよびShoulderPitchを一意に特定
    double shoulderRoll = (double)asin(elbowPosFromShoulder.y);
    double shoulderPitch = 0.0f;
    if (shoulderRoll > -1.5f && shoulderRoll < 1.5f)
      {
	//算術的チェック: X / Cos() が1越えてるとAsin関数がNaNを吐くのを防止
	if (fabs(elbowPosFromShoulder.x) > fabs(cos(shoulderRoll)))
	  {
	    shoulderPitch = 0.5f * 3.14f;
	  }
	else
	  {
	    shoulderPitch = (double)asin(elbowPosFromShoulder.x / (double)cos(shoulderRoll));
	    //追加処理: Pitchは360度分の自由度があるのだがAsinだと180度しか分解能ないので
	    //Z座標の正負情報を使って360度に対応させる
	    //具体的にはZ < 0 のときピッチは[-180, -90]か[90, 180]の範囲に入る
	    if (elbowPosFromShoulder.z < 0)
	      {
		if (shoulderPitch > 0)
		  {
		    shoulderPitch = (double)M_PI - shoulderPitch;
		  }
		else
		  {
		    shoulderPitch = (double)M_PI - shoulderPitch;
		  }
	      }
	  }
      }

    //ヒジのZベクトルつまりヒジの回転軸ベクトルの向き(と上のShoulderPitch)を用いてElbowYawを特定
    //このnoRollRElbowはElbowYaw=0の場合のヒジZベクトルの向きである(導出は手計算)
    geometry_msgs::Quaternion noRollRElbow;
    noRollRElbow.x  = (double)cos(shoulderPitch);
    noRollRElbow.z = -(double)sin(shoulderPitch);
      
    //noRollRElbowと実際のヒジZベクトルのズレはElbowYaw回転によって説明される、というノリで計算。
    //外積を用いているのはAcos関数だけだと回転方向が定まらない(どっち回転でもプラス扱いになる)ため
    geometry_msgs::Quaternion noRollRElbowCross = Cross(noRollRElbow, elbowZFromShoulder);
    double noRollRElbowProduct = Product(noRollRElbowCross, elbowPosFromShoulder);
    //std::cout << noRollRElbowProduct << std::endl;
    double elbowYaw = (double)asin(noRollRElbowProduct);//とても小さな値が入ってる
    
    //ヒジの曲がり具合を取得: 単に内積とって角度差を見ればOK
    double elbowCos = Product(wristY, elbowY);
    double elbowRoll = (double)acos(elbowCos);


    //0:left, 1:right 
    if (arm_id == 0)
      {
	angle->LShoulderPitch = -1*shoulderPitch;
	angle->LShoulderRoll = shoulderRoll;
	angle->LElbowYaw = elbowYaw;
	angle->LElbowRoll = -1*elbowRoll;
      }
    else
      {
	angle->RShoulderPitch = -1*shoulderPitch;
	angle->RShoulderRoll = -1*shoulderRoll;
	angle->RElbowYaw = elbowYaw;
	angle->RElbowRoll = elbowRoll;
      }
 
    //HACK: 正負の調整はPepperとKinectで回転方向の取り方が違うことに由来する。
    
    //Angles robot;

    
    //std::cout << robot.RShoulderPitch*180./M_PI << "," << robot.RShoulderRoll*180./M_PI << std::endl;
    //std::cout << robot.RElbowYaw*180./M_PI << "," << robot.RElbowRoll*180./M_PI << std::endl;
    //return robot;
  }


  void SetHip(humans_msgs::Body body, Angles *angle)
  {
    //1:spine_mid
    geometry_msgs::Quaternion spine = body.joints[1].orientation;
    geometry_msgs::Quaternion spineY;
    spineY = RotateBy(unitY, spine);
    //適当な実装: Kinect側に体を倒してるかどうかカメラ座標ベースで判定するだけ
    angle->HipPitch = (double)asin(spineY.z);
    
  }


  void SetHead(humans_msgs::Body body, Angles *angle)
  {
    geometry_msgs::Quaternion neck = body.joints[2].orientation;
    geometry_msgs::Quaternion spine = body.joints[20].orientation;
    
    //var neck = QuartanionFactory.FromVector4(body.JointOrientations[JointType.Neck].Orientation);
    //var spine = QuartanionFactory.FromVector4(body.JointOrientations[JointType.SpineShoulder].Orientation);
    geometry_msgs::Quaternion neckZ = RotateBy(unitZ, neck);
    
    //正規直交基底に使う
    geometry_msgs::Quaternion spineX = RotateBy(unitX, spine);
    //geometry_msgs::Quaternion spineY = RotateBy(unitY, spine);
    //geometry_msgs::Quaternion spineZ = RotateBy(unitZ, spine);;

    //geometry_msgs::Quaternion neckZfromSpine; 
    double neckZfromSpine = Product(neckZ, spineX);
  
    angle->HeadYaw = (double)asin(neckZfromSpine); 
  }




  void callback(const humans_msgs::Humans::ConstPtr& msg)
  {
    
    int p_num = msg->human.size();
    for(int i=0; i< p_num ; ++i)
      {
	//ジョイントが入ってないと計算不可
	if( msg->human[i].body.joints.size() )
	  {
	    Angles user = SetAnglesFromBody(msg->human[i].body);

	    cout << "head"<< user.HeadYaw << endl;
	    /*
	    long long t_id = msg->human[i].body.tracking_id;
	    //cout << "t_id: "<< t_id << endl;
	    vector<geometry_msgs::Point> joints;


	    //関節の三次元位置だけ取得し蓄積
	    joints = pnt_store( msg->human[i].body.joints );
	    JData jdata;
	    jdata.data = joints;
	    
	    //時間データが欲しかったら
	    jdata.sec = now_sec;
	    jdata.time = now_time;//now_sec;

	    //speaking
	    jdata.speaked = msg->human[i].body.is_speaked;
	    if(jdata.speaked)
	      now_speaking_id = i+1;
	    //顔の状態
	    jdata.face_prop = msg->human[i].body.face_info.properties;
	      
	    jpersons[t_id].push_back(jdata);
	    */
	  }
      }
    
  }

  
};






int main(int argc, char** argv)
{
  ros::init(argc, argv, "pepper_angle");
  PepperAngle pa;
  ros::spin();
  return 0;
}
