/*

2016.9.5-----
save data: kinect v2 joints position (x, y, z)
add save data: speaked
format: json


2016.3.29---
positionだけを保存

*/

#include "ros/ros.h"
#include "humans_msgs/Humans.h"

//ファイル処理
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include "picojson.h"

using namespace std;


class JData
{
public:
  vector<geometry_msgs::Point> data;
  double secs;
  bool speaked;
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
  
public:
  MyClass()
    :pnh("~")
  {
    sub = nh.subscribe("/humans/okao_server", 1, &MyClass::callback, this);
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
	    picojson::object obj;
	    picojson::array jcol;
	    for(int jj=0; jj<jit->second[i].data.size(); ++jj)
	      {
		picojson::array jjcol;
		jjcol.push_back(picojson::value(jit->second[i].data[jj].x));
		jjcol.push_back(picojson::value(jit->second[i].data[jj].y));
		jjcol.push_back(picojson::value(jit->second[i].data[jj].z));
		jcol.push_back(picojson::value(jjcol));
	      }
	    obj["jdata"] = picojson::value( jcol );
	    obj["time"] = picojson::value( jit->second[i].secs );
	    obj["speaked"] = picojson::value( jit->second[i].speaked );

	    rows.push_back(picojson::value(obj));
	  }
	objs["datas"] = picojson::value( rows );

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

  void callback(const humans_msgs::Humans::ConstPtr& msg)
  {
    double now_sec = ros::Time::now().toSec();
    int p_num = msg->human.size();    
    for(int i=0; i< p_num ; ++i)
      {
	//ジョイントが入ってないと計算不可
	if( msg->human[i].body.joints.size() )
	  {
	    long long t_id = msg->human[i].body.tracking_id;
	    //cout << "t_id: "<< t_id << endl;
	    vector<geometry_msgs::Point> joints;


	    //関節の三次元位置だけ取得し蓄積
	    joints = pnt_store( msg->human[i].body.joints );
	    JData jdata;
	    jdata.data = joints;
	    //時間データが欲しかったら
	    jdata.secs = now_sec;

	    //speaking
	    jdata.speaked = msg->human[i].body.is_speaked;
	      
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
