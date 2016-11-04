/*
2016.10.9
nose


2014.10.29------------------------
このヘッダがあるだけでJSON ---> ROS Messageできるようにする
*/


#pragma
#include "picojson.h"
#include <humans_msgs/Humans.h>
//#include <tf/transform_broadcaster.h>
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

  class Json2Msg{

  public:
    std::vector<std::string> named_joints;

    Json2Msg()
    {
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
    }
    

  
    void body(const KinectPack kinectPack , humans_msgs::Humans *kinect_msg, 
	      double cols, double rows, std::string camera_frame)
    {
      //cout <<"body"<<endl;
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
		  long long tracking_id
		    = std::atoll( objBody["trackingID"].get< std::string >().c_str() );
		  
		  bool isSpeaked = false;
		  
		  //if not speaking data
		  if (objBody["isSpeaked"].is< bool >())
		    {
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
		  humans_msgs::Joints nose;
		                  
		  nose = set_nose(head_position, tmp_human.body.face_info.rotation,
				  tracking_id, face_is_tracked);
		  //cout <<"set_nose_end"<<endl;
		  tmp_human.body.joints.push_back( nose );

		  //顔の角度更新
		  tmp_human.body.face_info.rotation.y = angle_map[ tracking_id ].pan;
		  tmp_human.body.face_info.rotation.p = angle_map[ tracking_id ].tilt;
		  
		  //FaceProp
		  humans_msgs::FaceProp face_prop_msg;
		  if (face_is_tracked)
		    {
		      picojson::object face_prop 
			= face_info["Property"].get<picojson::object>();		  
		      face_prop_msg = set_face_prop(face_prop);
		      //cout << face_prop_msg << endl;
		    }
		  else
		    {
		      face_prop_msg = unknown_face_prop();
		    }
		  tmp_human.body.face_info.properties = face_prop_msg;
		  
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



    humans_msgs::Joints set_nose(geometry_msgs::Point head_position, humans_msgs::Rotation rotation,
				 long long tracking_id, bool face_is_tracked)
    {
      //cout <<"set_nose"<<endl;
      humans_msgs::Joints nose;  

      double pan, tilt;
      if(face_is_tracked)
	{	
	  //ここにJoint noseの変換
	  pan = rotation.y;
	  tilt = rotation.p;
	  
	  //
	  nose.tracking_state = 2.0;
	  
	  //miscount[ tracking_id ] = 0;
	}
      else
	{
	  //もし見失ったら
	  //cout << "face miss" << endl;
	  pan = angle_map[ tracking_id ].pan;
	  tilt = angle_map[ tracking_id ].tilt;
	  nose.tracking_state = 0;
	  /*
	  int th = 5;
	  if(miscount[ tracking_id ] < th)
	    {	
	      pan = angle_map[ tracking_id ].pan;
	      tilt = angle_map[ tracking_id ].tilt;
	      nose.tracking_state = 1.0;
	    }
	  else
	    {
	      pan = 0;
	      tilt = 0;
	      nose.tracking_state = 0.0;
	    }
	  ++miscount[ tracking_id ];
	  */
	}
		  
      Angle angle;
      angle.pan = pan;
      angle.tilt = tilt;
      
      Angle angle_ave;
      
      //===Calc MovAve=and=Delete===//
      angle_stacks[tracking_id].push_back( angle );
      int angle_stacks_size = angle_stacks[tracking_id].size();

      //過去幾つのangleを使うか
      int stack_size = 3;

      //移動平均しないなら
      if( stack_size == 0 )
	{
	  angle_ave.pan = pan;
	  angle_ave.tilt = tilt;
	}
      else if (angle_stacks_size > stack_size)
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
		  
      nose.position = calc_nose_pos(head_position, angle_ave.pan,
				    angle_ave.tilt-(90.*M_PI/180.));
      nose.joint_name = "nose";

      return nose;
    }

    geometry_msgs::Point calc_nose_pos(geometry_msgs::Point head, double pan, double tilt)
    {
      //cout <<"calc_nose"<<endl;
      geometry_msgs::Point p;
      double l = 0.1;
      p.x = head.x+l*cos(pan)*sin(tilt);
      p.y = head.y+l*sin(pan)*sin(tilt);
      p.z = head.z+l*cos(tilt);
      return p;
    }


    humans_msgs::FaceProp set_face_prop(picojson::object face_prop)
    {  
      humans_msgs::FaceProp prop;
      prop.happy = face_prop["Happy"].get<string>();    
      prop.engaged = face_prop["Engaged"].get<string>();
      prop.wearing_glasses= face_prop["WearingGlasses"].get<string>();
      prop.right_eye_closed= face_prop["RightEyeClosed"].get<string>();
      prop.left_eye_closed= face_prop["LeftEyeClosed"].get<string>();
      prop.mouth_open= face_prop["MouthOpen"].get<string>();
      prop.mouth_moved= face_prop["MouthMoved"].get<string>();
      prop.looking_away= face_prop["LookingAway"].get<string>();      
      return prop;
    }

    humans_msgs::FaceProp unknown_face_prop()
    {  
      humans_msgs::FaceProp prop;
      prop.happy = "unknown";    
      prop.engaged = "unknown";
      prop.wearing_glasses= "unknown";
      prop.right_eye_closed= "unknown";
      prop.left_eye_closed= "unknown";
      prop.mouth_open= "unknown";
      prop.mouth_moved= "unknown";
      prop.looking_away= "unknown";      
      return prop;
    }

    
  };
}
