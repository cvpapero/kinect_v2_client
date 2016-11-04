/*
2016.10.9-----
顔角度からをnose joint定義

2014.10.29------------------------
このヘッダがあるだけでJSON ---> ROS Messageできるようにする
*/


#pragma
#include "picojson.h"
#include <humans_msgs/Humans.h>
#include <tf/transform_broadcaster.h>
//#include <tf2_ros/transform_broadcaster.h>
#include <cstdlib>

#include <unordered_map>   

using namespace std;



class Angle{
 public:
  double pan;
  double tilt;
};

//c++11じゃないと使えない？
unordered_map<long long, Angle> angle_map;
unordered_map<long long, int> miscount;
unordered_map<long long, vector<Angle> > angle_stacks;



namespace JsonToMsg{



  
  
  bool publishJointTF(ros::NodeHandle& nh, 
		      tf::TransformBroadcaster& br, tf::Transform& transform, 
		      humans_msgs::Human h, int j_n, std::string camera_frame)
  {
    //cout << "joint name: "<< j_name << ", (x, y, z) = " <<j.position.x <<", "<<j.position.y <<", "<<j.position.z <<endl;
    transform.setOrigin(tf::Vector3(h.body.joints[j_n].position.x,
				    h.body.joints[j_n].position.y, 
				    h.body.joints[j_n].position.z));

    transform.setRotation(tf::Quaternion(h.body.joints[j_n].orientation.x, 
					 h.body.joints[j_n].orientation.y, 
					 h.body.joints[j_n].orientation.z, 
					 h.body.joints[j_n].orientation.w));
    //transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    
    std::stringstream frame_id_stream;
    std::string frame_id;
    frame_id_stream << "/" << h.body.tracking_id <<"/" 
		    << h.body.joints[j_n].joint_name;
    frame_id = frame_id_stream.str();
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), 
					  camera_frame, frame_id));
    return true;
  }
  
  /*
  bool publishJointTF(ros::NodeHandle& nh, 
		      tf2_ros::TransformBroadcaster& br, tf::Transform& transform, 
		      humans_msgs::Human h, int j_n, std::string camera_frame)
  {
    //cout << "joint name: "<< j_name << ", (x, y, z) = " <<j.position.x <<", "<<j.position.y <<", "<<j.position.z <<endl;
    transform.setOrigin(tf::Vector3(h.body.joints[j_n].position.x,
				    h.body.joints[j_n].position.y, 
				    h.body.joints[j_n].position.z));

    transform.setRotation(tf::Quaternion(h.body.joints[j_n].orientation.x, 
					 h.body.joints[j_n].orientation.y, 
					 h.body.joints[j_n].orientation.z, 
					 h.body.joints[j_n].orientation.w));
    //transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    
    std::stringstream frame_id_stream;
    std::string frame_id;
    frame_id_stream << "/" << h.body.tracking_id <<"/" 
		    << h.body.joints[j_n].joint_name;
    frame_id = frame_id_stream.str();
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), 
					  camera_frame, frame_id));
    return true;
  }
  */


  geometry_msgs::Point calc_nose_pos(geometry_msgs::Point head, double pan, double tilt)
  {
    geometry_msgs::Point p;
    double l = 0.1;
    p.x = head.x+l*cos(pan)*sin(tilt);
    p.y = head.y+l*sin(pan)*sin(tilt);
    p.z = head.z+l*cos(tilt);
    return p;
  }
  
  void body(ros::NodeHandle& nh, 
	    const KinectPack kinectPack , humans_msgs::Humans *kinect_msg, 
	    double cols, double rows, std::string camera_frame)
  {


    std::vector<std::string> named_joints;
    named_joints.resize(25);
    named_joints[0] = "SpineBase";
    named_joints[1] = "SpineMid"; 
    named_joints[2] = "Neck";  
    named_joints[3] = "Head";
    named_joints[4] = "ShoulderLeft";
    named_joints[5] = "ElbowLeft";
    named_joints[6] = "WristLeft";
    named_joints[7] = "HandLeft";
    named_joints[8] = "ShoulderRight";
    named_joints[9] = "ElbowRight";
    named_joints[10] = "WristRight";
    named_joints[11] = "HandRight";
    named_joints[12] = "HipLeft";
    named_joints[13] = "KneeLeft";
    named_joints[14] = "AnkleLeft";
    named_joints[15] = "FootLeft";
    named_joints[16] = "HipRight";
    named_joints[17] = "KneeRight";
    named_joints[18] = "AnkleRight";
    named_joints[19] = "FootRight";
    named_joints[20] = "SpineShoulder"; 
    named_joints[21] = "HandTipLeft";
    named_joints[22] = "ThumbLeft";
    named_joints[23] = "HandTipRight";
    named_joints[24] = "ThumbRight";


    picojson::value v;
    std::string err;
    picojson::parse( v, kinectPack.bodies.jsonBodyInfo.begin(), 
		     kinectPack.bodies.jsonBodyInfo.end(), &err );

    if ( err.empty() )
      {
	picojson::object &objBodyInfo 
	  = v.get< picojson::object >();
	picojson::array arrayBody 
	  = objBodyInfo["bodies"].get< picojson::array >();
	//kinect_msg->human.resize(arrayBody.size());

	int index = 0;
	int people_num = 0;
	for( std::vector<picojson::value>::iterator itrBody = arrayBody.begin();
	     itrBody != arrayBody.end(); 
	     ++itrBody, ++index )
	  {
	    picojson::object &objBody 
	      = itrBody->get<picojson::object>();
	    bool isTracked
	      =  objBody["isTracked"].get< bool >() ;
	    
	    
	    if ( isTracked )
	      {
		//std::cout << objBody["trackingID"].get< std::string >() << std::endl;
		//long long tracking_id 
		//  = objBody["trackingID"].get< std::int64 >();
		
		long long tracking_id
		  = std::atoll( objBody["trackingID"].get< std::string >().c_str() );

		bool isSpeaked = false;

		//if not speaking data
		if (objBody["isSpeaked"].is< bool >()){
		  isSpeaked  =  objBody["isSpeaked"].get< bool >() ; //temp
		}
		
		//std::cout << "tracking_id[" << index << "]:" << tracking_id << std::endl;
		
		humans_msgs::Human tmp_human;
		tmp_human.body.is_tracked = isTracked;
		tmp_human.body.is_speaked = isSpeaked;
	        tmp_human.body.tracking_id = tracking_id;
		tmp_human.body.left_hand_state 
		  = objBody["leftHandState"].get< double >() ;
	        tmp_human.body.right_hand_state 
		  = objBody["rightHandState"].get< double >() ;

		picojson::array arrayJoint 
		  = objBody["Joints"].get<picojson::array>();

		int j_name = 0;
		geometry_msgs::Point head_position;
		for( std::vector<picojson::value>::iterator itrJoint = arrayJoint.begin(); 
		     itrJoint != arrayJoint.end(); ++itrJoint, ++j_name)
		  {
		    humans_msgs::Joints tmp_joint;
		    
		    picojson::object &objJoint = 
		      itrJoint->get<picojson::object>();
		    picojson::object &objPositionColorSpace 
		      = objJoint["PositionColorSpace"].get<picojson::object>();
		    tmp_joint.position_color_space.x 
		      = (int)objPositionColorSpace["X"].get<double>() * cols;
		    tmp_joint.position_color_space.y 
		      = (int)objPositionColorSpace["Y"].get<double>() * rows;
		    
		    picojson::object &objPosition = 
		      objJoint["Position"].get<picojson::object>();
		    tmp_joint.position.y 
		      = (double)objPosition["X"].get<double>();
		    tmp_joint.position.z 
		      = (double)objPosition["Y"].get<double>();
		    tmp_joint.position.x 
		      = (double)objPosition["Z"].get<double>();

		    picojson::object &objOrientation = 
		      objJoint["Orientation"].get<picojson::object>();
		    tmp_joint.orientation.x 
		      = (double)objOrientation["X"].get<double>();
		    tmp_joint.orientation.y 
		      = (double)objOrientation["Y"].get<double>();
		    tmp_joint.orientation.z 
		      = (double)objOrientation["Z"].get<double>();
		    tmp_joint.orientation.w 
		      = (double)objOrientation["W"].get<double>();

		    tmp_joint.tracking_state 
		      = objJoint["trackingState"].get<double>();
		    tmp_joint.joint_name = named_joints[j_name];

		    tmp_human.body.joints.push_back( tmp_joint );
		    //tmp_human.body.b
		    //tf変換
		    /*
		    if(tmp_joint.orientation.x || tmp_joint.orientation.y 
		       || tmp_joint.orientation.z || tmp_joint.orientation.w)
		      {
			publishJointTF(nh, br, transform, tmp_human, 
				       j_name, camera_frame);
		      }
		    */
		    
		    //Get head position
		    if(named_joints[j_name]=="Head")
		      {
			head_position.x = tmp_joint.position.x;
			head_position.y = tmp_joint.position.y;
			head_position.z = tmp_joint.position.z;
		      }
		    
		  }

		   
		picojson::object face_info
		  = objBody["FaceInfo"].get<picojson::object>();
			
		picojson::object rotation 
		  = face_info["Rotation"].get<picojson::object>();
		
		tmp_human.body.face_info.rotation.r = (double)rotation["R"].get<double>();
		tmp_human.body.face_info.rotation.p = (double)rotation["P"].get<double>();
		tmp_human.body.face_info.rotation.y = (double)rotation["Y"].get<double>();


		//角度を保存する
		//なぜならheadは動いているのにnoseが固定されてたらおかしい
		//角度を保存するとheadとnoseの位置関係を覚えられる

		bool face_is_tracked =  face_info["isTracked"].get< bool >();
		double pan, tilt;
		if(face_is_tracked)
		  {	
		    //ここにJoint noseの変換
		    pan = tmp_human.body.face_info.rotation.y;
		    tilt = tmp_human.body.face_info.rotation.p;
		    
		    miscount[ tracking_id ] = 0;

		  }
		else
		  {
		    //もし見失ったら
		    //cout << "face miss" << endl;
		    int th = 5;
		    if(miscount[ tracking_id ] < th)
		      {	
			pan = angle_map[ tracking_id ].pan;
			tilt = angle_map[ tracking_id ].tilt; 
		      }
		    else
		      {
			pan = 0;
			tilt = 0;
		      }
		    ++miscount[ tracking_id ];
		  }


		Angle angle;
		angle.pan = pan;
		angle.tilt = tilt;
		
		Angle angle_ave;

		//===Calc MovAve=and=Delete===//
		angle_stacks[tracking_id].push_back( angle );
		int angle_stacks_size = angle_stacks[tracking_id].size();
		int stack_size = 3;
		if (angle_stacks_size > stack_size)
		  {
		    //cout<<nose_stacks[human.body.tracking_id].at(0)<<endl;
		    double sum_p = 0, sum_t = 0;
		    
		    for(int i = 0; i < angle_stacks_size; ++i)
		      {
			sum_p += angle_stacks[tracking_id][i].pan;
			sum_t += angle_stacks[tracking_id][i].tilt;
		      }
		    angle_ave.pan = sum_p/angle_stacks_size;
		    angle_ave.tilt = sum_t/angle_stacks_size;

		    angle_stacks[tracking_id].erase(angle_stacks[tracking_id].begin());
		  }

		angle_map[ tracking_id ].pan = angle_ave.pan;
		angle_map[ tracking_id ].tilt = angle_ave.tilt;
	    		
		humans_msgs::Joints nose;
		nose.position = calc_nose_pos(head_position, angle_ave.pan, angle_ave.tilt-(90.*M_PI/180.));
		nose.joint_name = "nose";
		tmp_human.body.joints.push_back( nose );
		
		kinect_msg->human.push_back( tmp_human );	
		++people_num;
	      } 

	  }
	kinect_msg->num = people_num;
      }
    else
      {
	std::cerr << __FUNCTION__ << ":" << err << std::endl;
      } 
  } 
}
