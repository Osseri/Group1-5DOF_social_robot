
/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <sstream>
#include <fstream>
#include "social_robot_arm_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace social_robot_arm_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
  init_argc_(argc),
  init_argv_(argv)
{}

QNode::~QNode() {
  if(ros::isStarted())
  {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init() {
  ros::init(init_argc_,init_argv_,"social_robot_arm_gui");
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  curr_time = ros::Time::now();
  tar_time = curr_time + ros::Duration(24*60*60*30);
  last_record_time = curr_time + ros::Duration(24*60*60*30);

  // Add your ros communications here.
  set_ctrl_module_pub_ = n.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1);

  joint_pose_msg_pub_ = n.advertise<social_robot_arm_sdk::JointPose>("/robotis/goal_joint_pose", 0);
  head_waist_pose_msg_pub_ = n.advertise<social_robot_arm_sdk::JointPose>("/robocare/goal_joint_pose", 0);
  kinematics_pose_msg_pub_ = n.advertise<social_robot_arm_sdk::KinematicsPose>("/robotis/goal_kinematics_pose", 0);

  get_joint_pose_client_ = n.serviceClient<social_robot_arm_sdk::GetJointPose>("/robotis/get_joint_pose", 0);
  get_kinematics_pose_client_ = n.serviceClient<social_robot_arm_sdk::GetKinematicsPose>("/robotis/get_kinematics_pose", 0);

  wr_pos_pub_ = n.advertise<std_msgs::Float64>("/social_robot/Waist_Roll_position/command", 0);
  wp_pos_pub_ = n.advertise<std_msgs::Float64>("/social_robot/Waist_Pitch_position/command", 0);
  hy_pos_pub_ = n.advertise<std_msgs::Float64>("/social_robot/Head_Yaw_position/command", 0);
  hp_pos_pub_ = n.advertise<std_msgs::Float64>("/social_robot/Head_Pitch_position/command", 0);

  motion_pub_ = n.advertise<social_robot_arm_sdk::PlayMotion>("/arm/play_motion", 0);
  present_joint_msg_sub_ = n.subscribe("/social_robot/joint_states", 5, &QNode::presentJointCallback, this);

  start();
  return true;
}

void QNode::run() {

  ros::Rate loop_rate(50);

  while ( ros::ok() )
  {
    ros::spinOnce();

    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::log( const LogLevel &level, const std::string &msg, std::string sender)
{
  logging_model_.insertRows(logging_model_.rowCount(),1);
  std::stringstream logging_model_msg;

  std::stringstream _sender;
  _sender << "[" << sender << "] ";

  switch ( level ) {
  case(Debug) : {
    ROS_DEBUG_STREAM(msg);
    logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << _sender.str() << msg;
    break;
  }
  case(Info) : {
    ROS_INFO_STREAM(msg);
    logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << _sender.str() << msg;
    break;
  }
  case(Warn) : {
    ROS_WARN_STREAM(msg);
    logging_model_msg << "[WARN] [" << ros::Time::now() << "]: " << _sender.str() <<msg;
    break;
  }
  case(Error) : {
    ROS_ERROR_STREAM(msg);
    logging_model_msg << "<ERROR> [" << ros::Time::now() << "]: " << _sender.str() <<msg;
    break;
  }
  case(Fatal) : {
    ROS_FATAL_STREAM(msg);
    logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << _sender.str() <<msg;
    break;
  }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model_.setData(logging_model_.index(logging_model_.rowCount()-1),new_row);
  Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}



void QNode::sendJointPoseMsg(social_robot_arm_sdk::JointPose msg)
{
  joint_pose_msg_pub_.publish( msg );

  log( Info , "Send Joint Pose Msg" );
}

void QNode::sendKinematicsPoseMsg( social_robot_arm_sdk::KinematicsPose msg )
{
  kinematics_pose_msg_pub_.publish( msg );

  log( Info , "Send Kinematics Pose Msg" );
}

void QNode::getJointPose()
{
//  log( Info , "Get Current Joint Pose" );

//   social_robot_arm_sdk::GetJointPose get_joint_pose;

//   // request

//   // response
//   if ( get_joint_pose_client_.call ( get_joint_pose ) )
//   {
//     social_robot_arm_sdk::JointPose joint_pose;

// //    log( Info , "Get Joint Pose Client Call" );

//     for ( int i = 0; i < get_joint_pose.response.pose.pose.name.size(); i++ )
//     {
//       joint_pose.pose.name.push_back( get_joint_pose.response.pose.pose.name[i] );
//       joint_pose.pose.position.push_back( get_joint_pose.response.pose.pose.position[i] );
// //      ROS_INFO("Get Current Joint Position[%d] : %lf", i, get_joint_pose.response.pose.pose.position[i]);
//     }

    
//     Q_EMIT updateCurrentJointPose( joint_pose );
//   }
//   else
//     log(Error, "fail to get joint pose.");

  social_robot_arm_sdk::JointPose joint_pose;
  for ( int i = 0; i < joint_state.name.size(); i++ )
  {
    joint_pose.pose.name.push_back( joint_state.name[i] );
    joint_pose.pose.position.push_back( joint_state.position[i] );
//      ROS_INFO("Get Current Joint Position[%d] : %lf", i, get_joint_pose.response.pose.pose.position[i]);
  }
  Q_EMIT updateCurrentJointPose( joint_pose );
}


void QNode::getKinematicsPose ( std::string group_name )
{
//  log( Info , "Get Current Kinematics Pose" );

  social_robot_arm_sdk::GetKinematicsPose get_kinematics_pose;

  // request
  get_kinematics_pose.request.name = group_name;

  // response
  if ( get_kinematics_pose_client_.call( get_kinematics_pose ) )
  {
    social_robot_arm_sdk::KinematicsPose kinematcis_pose;

    kinematcis_pose.name = get_kinematics_pose.request.name;
    kinematcis_pose.pose = get_kinematics_pose.response.pose.pose;


    if(group_name=="left_arm")
      Q_EMIT updateLeftCurrentKinematicsPose( kinematcis_pose );
    else if(group_name=="right_arm")
      Q_EMIT updateRightCurrentKinematicsPose( kinematcis_pose );
  }
  else
    log(Error, "fail to get kinematcis pose.");
}

void QNode::sendSetModeMsg()
{
  std_msgs::String str_msg;
  str_msg.data = "arm_control_module";

  set_ctrl_module_pub_.publish(str_msg);

  return;
}

// for test
void QNode::sendSetMode(const std::string &mode_name)
{
  std_msgs::String str_msg;
  str_msg.data = mode_name;

  set_ctrl_module_pub_.publish(str_msg);

  std::stringstream ss;
  ss << "send Set Mode : " << mode_name;
  log(Info, ss.str());

  return;
}

void QNode::sendPlayMotionMsg(std::vector<std::string> file_times, std::vector<std::string> hw_file_times, std::string file_name)
{
  // motion msg
  social_robot_arm_sdk::PlayMotion motion_msg;

  // sensor_msgs::JointState joint = joint_state;
  // motion_msg.pose.push_back(joint);
  // motion_msg.delta_time_ms.push_back(0);
  double tcount = 0;
  for(int ix = 0; ix < file_times.size(); ix++)
  {
    std::string file_time = file_times[ix];
    std::string delimiter = ",";

    // std::cout << file_time << std::endl;
    ROS_INFO("arm file, time %s",file_time.c_str());
    int pos = file_time.find(delimiter);
    std::string path = file_time.substr(0, pos);
    file_time.erase(0,pos+1);
    std::string time_string = file_time;

    std::string file_path = ros::package::getPath("social_robot_arm_gui") + "/arm/"+path;
    double delta_time = std::stod(time_string);
    tcount = tcount + delta_time;
    motion_msg.delta_time_ms.push_back(delta_time);

    sensor_msgs::JointState tar_joints;

    YAML::Node doc;
    try
    {
      // load yaml
      doc = YAML::LoadFile(file_path.c_str());
    } catch (const std::exception& e)
    {
      ROS_ERROR("Fail to load yaml file. %s",file_path.c_str());
      return;
    }

    social_robot_arm_sdk::JointPose msg;

    // parse movement time
    // double mov_time = doc["mov_time"].as<double>();
    // msg.mov_time = mov_time;

    // parse target pose
    YAML::Node tar_pose_node = doc["tar_pose"];
    for (YAML::iterator it = tar_pose_node.begin(); it != tar_pose_node.end(); ++it)
    {
      std::string joint_name = it->first.as<std::string>();
      double value = it->second.as<double>();

      tar_joints.name.push_back(joint_name);
      double rad = value * DEG2RAD;
      tar_joints.position.push_back(rad);
      double v_rad = rad * 1000 / delta_time;
      tar_joints.velocity.push_back(0);
    }

      // tar_joints.name.push_back(joint_name[joint_index]);

      // int deg = rand() % 180 - 90;
      // double rad = deg * DEG2RAD;
      // tar_joints.position.push_back(rad);
      // double v_rad = rad * 1000 / delta_time;
      // tar_joints.velocity.push_back(v_rad);

    motion_msg.pose.push_back(tar_joints);
  }

  boost::thread(boost::bind(&QNode::sendPlayMotionMsgHW, this, hw_file_times));
  // sendPlayMotionMsgHW(hw_file_times);
  motion_pub_.publish(motion_msg);
  joint_vector.clear();
  if(file_name != "") {
    tar_file = ros::package::getPath("social_motion_res") + "/joint/"+file_name;
    std::cout << "tcount : " << tcount << std::endl;
    tar_time = ros::Time::now()+ros::Duration(tcount/1000);
    last_record_time = ros::Time::now();
  } else {
    // tar_file = ros::package::getPath("social_robot_arm_gui") + "/motion/test.joint";
    // tar_time = ros::Time::now()+ros::Duration(tcount/1000);
    // last_record_time = ros::Time::now()-ros::Duration(1000);
  }
  // ss << "Send msg for playing motion : " << num_motion;
}

void QNode::sendPlayMotionMsgHW(std::vector<std::string> file_times)
{
  sensor_msgs::JointState last_joint = joint_state;

  for(int ix = 0; ix < file_times.size(); ix++)
  {
    std::string file_time = file_times[ix];
    std::string delimiter = ",";

    ROS_INFO("head waist file, time %s",file_time.c_str());
    std::cout << file_time << std::endl;
    ROS_INFO("motion file, time %s",file_time.c_str());
    int pos = file_time.find(delimiter);
    std::string path = file_time.substr(0, pos);
    file_time.erase(0,pos+1);
    std::string time_string = file_time;
    std::string file_path = ros::package::getPath("social_robot_arm_gui") + "/head_waist/"+path;
    double delta_time = std::stod(time_string);


    social_robot_arm_sdk::JointPose msg;
    sensor_msgs::JointState tar_joints;

    YAML::Node doc;
    try
    {
      // load yaml
      doc = YAML::LoadFile(file_path.c_str());
    } catch (const std::exception& e)
    {
      ROS_ERROR("Fail to load yaml file. %s",file_path.c_str());
      return;
    }

    msg.mov_time = delta_time;

    // parse target pose
    YAML::Node tar_pose_node = doc["tar_pose"];
    for (YAML::iterator it = tar_pose_node.begin(); it != tar_pose_node.end(); ++it)
    {
      std::string joint_name = it->first.as<std::string>();
      double value = it->second.as<double>();

      tar_joints.name.push_back(joint_name);
      double rad = value * DEG2RAD;
      tar_joints.position.push_back(rad);
      double v_rad = rad * 1000 / delta_time;
      tar_joints.velocity.push_back(v_rad);
    }
    msg.pose = tar_joints;
    sendHeadWaist(msg);
  }

}

void QNode::sendHeadWaistThread(social_robot_arm_sdk::JointPose tar_joints) {
  boost::thread(boost::bind(&QNode::sendHeadWaist, this, tar_joints));
}

void QNode::sendHeadWaist(social_robot_arm_sdk::JointPose tar_joints) {
  // std::cout << "sendHeadWaist" << std::endl;

  double curr_angles[4];
  double tar_angles[4];
  double expected_tic[4];
  sensor_msgs::JointState curr_joint = joint_state;
  double mov_time = tar_joints.mov_time;
  for(int jx = 0; jx < curr_joint.name.size() ; jx++) {
    std::string curr_name = curr_joint.name.at(jx);
    double curr_value = curr_joint.position[jx];
    if(curr_name == "Waist_Roll") {
      curr_angles[0] = curr_value;
    } else if(curr_name == "Waist_Pitch") {
      curr_angles[1] = curr_value;
    } else if(curr_name == "Head_Yaw") {
      curr_angles[2] = curr_value;
    } else if(curr_name == "Head_Pitch") {
      curr_angles[3] = curr_value;
    }

  }
  for(int jx = 0; jx < tar_joints.pose.name.size() ; jx++) {
    std::string tar_name = tar_joints.pose.name.at(jx);
    double tar_value = tar_joints.pose.position[jx];
    if(tar_name == "Waist_Roll") {
      tar_angles[0] = tar_value;
    } else if(tar_name == "Waist_Pitch") {
      tar_angles[1] = tar_value;
    } else if(tar_name == "Head_Yaw") {
      tar_angles[2] = tar_value;
    } else if(tar_name == "Head_Pitch") {
      tar_angles[3] = tar_value;
    }
  }
  for(int jx = 0; jx < 4; jx++) {
    expected_tic[jx] = (tar_angles[jx] - curr_angles[jx]) * 10 / mov_time;
  }
  // std::cout << "mov_time : " << mov_time << std::endl;
  // std::cout << "[0]" << curr_angles[0] << " , " << tar_angles[0] << " , " << expected_tic[0] << std::endl;
  // std::cout << "[1]" << curr_angles[1] << " , " << tar_angles[1] << " , " << expected_tic[1] << std::endl;
  // std::cout << "[3]" << curr_angles[3] << " , " << tar_angles[3] << " , " << expected_tic[3] << std::endl;

  ros::Rate rate(100);
  for(double tx = 0; tx < mov_time/10; tx = tx + 1) {
    // std::cout << tx << "[2]" << curr_angles[2] << " , " << tar_angles[2] << " , " << expected_tic[2] << std::endl;
    curr_angles[0] = curr_angles[0] + expected_tic[0];
    curr_angles[1] = curr_angles[1] + expected_tic[1];
    curr_angles[2] = curr_angles[2] + expected_tic[2];
    curr_angles[3] = curr_angles[3] + expected_tic[3];
    
    std_msgs::Float64 msg0;
    msg0.data = curr_angles[0];
    wr_pos_pub_.publish(msg0);
    
    std_msgs::Float64 msg1;
    msg1.data = curr_angles[1];
    wp_pos_pub_.publish(msg1);

    std_msgs::Float64 msg2;
    msg2.data = curr_angles[2];
    hy_pos_pub_.publish(msg2);
    
    std_msgs::Float64 msg3;
    msg3.data = curr_angles[3];
    hp_pos_pub_.publish(msg3);

    rate.sleep();
  }

}


void QNode::presentJointCallback(const sensor_msgs::JointState &msg) {
  joint_state = msg;
  curr_time = ros::Time::now();
  if(curr_time - last_record_time >= ros::Duration(0.02) && curr_time < tar_time) {
    ROS_ERROR("seq. %d",msg.header.seq);
    joint_vector.push_back(msg);
    last_record_time = curr_time;
  } else if(curr_time > last_record_time && curr_time >= tar_time) {
    ROS_ERROR("record end. %s",tar_file.c_str());
    tar_time = curr_time;
    last_record_time = curr_time + ros::Duration(24*60*60*30);
    std::ofstream ss(tar_file.data());
    ss << std::fixed << std::setprecision(3);
    if( ss.is_open() ){
      for(int ix = 0; ix < joint_vector.size(); ix++) {
        double angles[16];
        sensor_msgs::JointState joint = joint_vector.at(ix);
        for(int jx = 0; jx < joint.name.size() ; jx++) {
          std::string name = joint.name.at(jx);
          if(name == "LShoulder_Pitch") {
            angles[0] = joint.position[jx];
          } else if(name == "LShoulder_Roll") {
            angles[1] = joint.position[jx];
          } else if(name == "LElbow_Pitch") {
            angles[2] = joint.position[jx];
          } else if(name == "LElbow_Yaw") {
            angles[3] = joint.position[jx];
          } else if(name == "LWrist_Pitch") {
            angles[4] = joint.position[jx];
          } else if(name == "LFinger") {
            angles[5] = joint.position[jx];
          } else if(name == "RShoulder_Pitch") {
            angles[6] = joint.position[jx];
          } else if(name == "RShoulder_Roll") {
            angles[7] = joint.position[jx];
          } else if(name == "RElbow_Pitch") {
            angles[8] = joint.position[jx];
          } else if(name == "RElbow_Yaw") {
            angles[9] = joint.position[jx];
          } else if(name == "RWrist_Pitch") {
            angles[10] = joint.position[jx];
          } else if(name == "RFinger") {
            angles[11] = joint.position[jx];
          } else if(name == "Waist_Roll") {
            angles[12] = joint.position[jx];
          } else if(name == "Waist_Pitch") {
            angles[13] = joint.position[jx];
          } else if(name == "Head_Pitch") {
            angles[14] = joint.position[jx];
          } else if(name == "Head_Yaw") {
            angles[15] = joint.position[jx];
          }
        }
        ss << angles[0];
        for(int ix = 1; ix < 16; ix++) {
          ss << "," << angles[ix];
        }
        ss << "\n";
      }
      ss.close();
    }
    // tar_file
  }
}

}  // namespace social_robot_arm_gui
