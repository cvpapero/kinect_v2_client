/*
2016.10.10----
kinect_v2_clientからの情報をすべて保存


2016.9.5-----
save data: kinect v2 joints position (x, y, z)
add save data: speaked
format: json


2016.3.29---
positionだけを保存

*/

#include "ros/ros.h"
#include "humans_msgs/Humans.h"
#include "humans_msgs/FaceProp.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//ファイル処理
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include "picojson.h"

//time
#include <time.h>
#include <sys/time.h>
#include <sstream>

using namespace std;

class JData
{
public:
  vector<geometry_msgs::Point> data;
  double time;
  double sec;
  bool speaked;
  humans_msgs::FaceProp face_prop;
};


class MyClass
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  ros::Subscriber sub;

  map< long long, vector<JData> > jpersons;

  string joint_filename;
  string output_filename;
  stringstream out_file;

  double pre_sec;
  int count;

  // time_t構造体を定義．1970年1月1日からの秒数を格納するもの
  struct timeval myTime;
  // tm構造体を定義．年月日時分秒をメンバ変数に持つ構造体
  struct tm *time_st;  

  //typedef message_filters::Subscriber< humans_msgs::Humans > Kinect1Subscriber;
  typedef message_filters::Subscriber< humans_msgs::Humans > KinectSubscriber;
  
  KinectSubscriber kinect1_sub_;
  KinectSubscriber kinect2_sub_;
  
  typedef message_filters::sync_policies::ApproximateTime<
    humans_msgs::Humans, humans_msgs::Humans
    > MySyncPolicy;
  
  message_filters::Synchronizer< MySyncPolicy > sync;

  
public:
  MyClass()
    :pnh("~"),
     kinect1_sub_( nh, "/humans/kinect_v2/1", 100 ),
     kinect2_sub_( nh, "/humans/kinect_v2/2", 100 ),
     sync( MySyncPolicy( 10 ), kinect1_sub_, kinect2_sub_ )
  {

    sync.registerCallback( boost::bind( &MyClass::callback, this, _1, _2 ) );
    
    //sub = nh.subscribe("/humans/kinect_v2", 1, &MyClass::callback, this);
    pnh.param<std::string>("output", output_filename, "test");
    out_file << output_filename << ".json";
    pre_sec = ros::Time::now().toSec();
    count = 0;
  }
  
  ~MyClass()
  {
    output_table();
  }


  //データ構造を持つので、jsonを使って保存する
  void output_table()
  {
    string output;
    picojson::array users;
    map< long long, vector<JData> >::iterator jit =  jpersons.begin();

    while(jit != jpersons.end())
      { 
	picojson::object objs;
	picojson::array rows;
	cout << "data_size(): "<< jit->second.size()
	     << ", demension_size(): "<< jit->second[0].data.size() << endl;
	for(int i=0; i<jit->second.size(); ++i)
	  {
	    
	    picojson::array jcol;
	    for(int jj=0; jj<jit->second[i].data.size(); ++jj)
	      {
		picojson::array jjcol;
		jjcol.push_back(picojson::value(jit->second[i].data[jj].x));
		jjcol.push_back(picojson::value(jit->second[i].data[jj].y));
		jjcol.push_back(picojson::value(jit->second[i].data[jj].z));
		jcol.push_back(picojson::value(jjcol));
	      }

	    picojson::object obj;
	    obj["joints"] = picojson::value( jcol );

	    obj["sec"] = picojson::value( jit->second[i].sec );
	    obj["time"] = picojson::value( jit->second[i].time );
	    
	    obj["speaked"] = picojson::value( jit->second[i].speaked );

	    //ここに顔の状態をjson形式で保存
	    picojson::object prop;
	    prop = insert_prop( jit->second[i].face_prop );
	    obj["properties"] = picojson::value( prop );
	    

	    //
	    rows.push_back( picojson::value(obj) );
	  }
	objs["data"] = picojson::value( rows );

	stringstream ss;
	ss << jit->first;
	cout << "t_id:" << ss.str() << endl;
	objs["t_id"] = picojson::value( ss.str() );

	users.push_back(picojson::value(objs));

	++jit;
      }

    picojson::value data = picojson::value(users);
    output = data.serialize();

    ofstream ofs(out_file.str().c_str());
    ofs << output;

				  
				  
    cout << "output: "<<out_file.str()<<endl;
  }


  picojson::object insert_prop(humans_msgs::FaceProp prop)
  {
    picojson::object dst;
    
    dst["happy"] = picojson::value( prop.happy );
    dst["engaged"] = picojson::value( prop.engaged );
    dst["wearing_glasses"] = picojson::value( prop.wearing_glasses );
    dst["right_eye_closed"] = picojson::value( prop.right_eye_closed );
    dst["left_eye_closed"] = picojson::value( prop.left_eye_closed );
    dst["mouth_open"] = picojson::value( prop.mouth_open );
    dst["mouth_moved"] = picojson::value( prop.mouth_moved );
    dst["looking_away"] = picojson::value( prop.looking_away );
    return dst;
  }


  string add_zero(int d)
  {
    stringstream s_d;
    d < 10 ? s_d << 0 << d : s_d << d;
    return s_d.str();
  }
  
  
  void callback(
		const humans_msgs::Humans::ConstPtr& msg1,
		const humans_msgs::Humans::ConstPtr& msg2)
  {
    double now_sec = ros::Time::now().toSec();
    
    vector<humans_msgs::Human> msgs;
    for(int i = 0; i<msg1->human.size(); ++i)
      msgs.push_back(msg1->human[i]);
    
    for(int i = 0; i<msg2->human.size(); ++i)
      msgs.push_back(msg2->human[i]);
    //msgs.push_back(msg2);
    int p_num = msgs.size(); //+msg2->human.size();

    double now_time;
    
    // 現在時刻を取得してmyTimeに格納．通常のtime_t構造体とsuseconds_tに値が代入
    gettimeofday(&myTime, NULL);
    // time_t構造体を現地時間でのtm構造体に変換
    time_st = localtime(&myTime.tv_sec);    

    /*
    int day = time_st->tm_mday;
    stringstream s_day;
    day < 10 ? s_day << 0 << day : s_mon << day;

    int hour = time_st->tm_hour;
    stringstream s_hour;
    hour < 10 ? s_hour << 0 << hour : s_hour << hour;
    */
    string mon = add_zero(time_st->tm_mon+1);
    string day = add_zero(time_st->tm_mday);
    string hour = add_zero(time_st->tm_hour);
    string min = add_zero(time_st->tm_min);
    string sec = add_zero(time_st->tm_sec);
    
    stringstream ss;   
    ss << time_st->tm_year+1900 << mon << day
       << hour << min << sec
       <<"."<<myTime.tv_usec;
    now_time = stod(ss.str());
    
    
    for(int i=0; i< p_num ; ++i)
      {
	//ジョイントが入ってないと計算不可
	if( msgs[i].body.joints.size() )
	  {
	    long long t_id = msgs[i].body.tracking_id;
	    //cout << "t_id: "<< t_id << endl;
	    vector<geometry_msgs::Point> joints;


	    //関節の三次元位置だけ取得し蓄積
	    joints = pnt_store( msgs[i].body.joints );
	    JData jdata;
	    jdata.data = joints;
	    
	    //時間データが欲しかったら
	    jdata.sec = now_sec;
	    jdata.time = now_time;//now_sec;

	    //speaking
	    jdata.speaked = msgs[i].body.is_speaked;

	    //顔の状態
	    jdata.face_prop = msgs[i].body.face_info.properties;
	      
	    jpersons[t_id].push_back(jdata);
	  }
      }
    
    double diff_sec = now_sec - pre_sec;
    cout << "count:" << count  <<", person:" << p_num << ", fps:" << 1/diff_sec << endl;
    
    pre_sec = now_sec;
    ++count;
  }

  vector<geometry_msgs::Point> pnt_store(vector<humans_msgs::Joints> jt)
  {
    vector<geometry_msgs::Point> jps;
    for(int i=0; i<jt.size(); ++i)
      {
	jps.push_back( jt[i].position );
      }
    return jps;
  }




  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_angle");
  MyClass mc;
  ros::spin();
  return 0;
}
