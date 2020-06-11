/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include "social_robot_arm_sdk/arm_control_module.h"

using namespace social_robot_arm;

ArmControlModule::ArmControlModule()
  : control_cycle_sec_(0.01),
    is_moving_(false),
    goal_initialize_(false),
    joint_control_initialize_(false)
{
  enable_       = false;
  module_name_  = "arm_control_module";
  control_mode_ = robotis_framework::PositionControl;
  control_type_ = NONE;

  social_robot_arm_kd_ = new SocialRobotArmKinematics();

  //arm
  result_["LShoulder_Pitch"]  = new robotis_framework::DynamixelState();
  result_["LShoulder_Roll"]  = new robotis_framework::DynamixelState();
  result_["LElbow_Pitch"]  = new robotis_framework::DynamixelState();
  result_["LElbow_Yaw"]  = new robotis_framework::DynamixelState();
  result_["LWrist_Pitch"]  = new robotis_framework::DynamixelState();
  result_["RShoulder_Pitch"] = new robotis_framework::DynamixelState();
  result_["RShoulder_Roll"] = new robotis_framework::DynamixelState();
  result_["RElbow_Pitch"] = new robotis_framework::DynamixelState();
  result_["RElbow_Yaw"] = new robotis_framework::DynamixelState();
  result_["RWrist_Pitch"] = new robotis_framework::DynamixelState();

  //hand
  result_["LFinger"] = new robotis_framework::DynamixelState();
  result_["RFinger"] = new robotis_framework::DynamixelState();


  joint_name_to_id_["LShoulder_Pitch"]  = 1;
  joint_name_to_id_["LShoulder_Roll"]  = 2;
  joint_name_to_id_["LElbow_Pitch"]  = 3;
  joint_name_to_id_["LElbow_Yaw"]  = 4;
  joint_name_to_id_["LWrist_Pitch"]  = 5;
  joint_name_to_id_["LFinger"] = 6;
  joint_name_to_id_["RShoulder_Pitch"] = 7;
  joint_name_to_id_["RShoulder_Roll"] = 8;
  joint_name_to_id_["RElbow_Pitch"] = 9;
  joint_name_to_id_["RElbow_Yaw"] = 10;
  joint_name_to_id_["RWrist_Pitch"] = 11;
  joint_name_to_id_["RFinger"] = 12;


  joint_id_to_name_[1]  = "LShoulder_Pitch";
  joint_id_to_name_[2]  = "LShoulder_Roll";
  joint_id_to_name_[3]  = "LElbow_Pitch";
  joint_id_to_name_[4]  = "LElbow_Yaw";
  joint_id_to_name_[5]  = "LWrist_Pitch";
  joint_id_to_name_[6]  = "LFinger";
  joint_id_to_name_[7]  = "RShoulder_Pitch";
  joint_id_to_name_[8]  = "RShoulder_Roll";
  joint_id_to_name_[9]  = "RElbow_Pitch";
  joint_id_to_name_[10]  = "RElbow_Yaw";
  joint_id_to_name_[11] = "RWrist_Pitch";
  joint_id_to_name_[12]  = "RFinger";


  /* parameter */
  number_of_joints_ = NUM_OF_JOINTS;

  curr_joint_accel_.resize(number_of_joints_, 0.0);
  curr_joint_vel_.resize(number_of_joints_, 0.0);
  curr_joint_pos_.resize(number_of_joints_, 0.0);

  des_joint_accel_.resize(number_of_joints_, 0.0);
  des_joint_vel_.resize(number_of_joints_, 0.0);
  des_joint_pos_.resize(number_of_joints_, 0.0);

  goal_joint_accel_.resize(number_of_joints_, 0.0);
  goal_joint_vel_.resize(number_of_joints_, 0.0);
  goal_joint_pos_.resize(number_of_joints_, 0.0);

  des_larm_pos_.resize(3, 0.0);
  des_larm_vel_.resize(3, 0.0);
  des_larm_accel_.resize(3, 0.0);
  des_larm_Q_.resize(4, 0.0);;

  des_rarm_pos_.resize(3, 0.0);
  des_rarm_vel_.resize(3, 0.0);
  des_rarm_accel_.resize(3, 0.0);
  des_rarm_Q_.resize(4, 0.0);;

  pre_larm_pos_.resize(3, 0.0);
  pre_larm_Q_.resize(4, 0.0);;
  pre_rarm_pos_.resize(3, 0.0);
  pre_rarm_Q_.resize(4, 0.0);;

  // Body Parameter
  body_pos_.resize(3,0.0);
  body_pos_[2] = 0.0;

  body_Q_.resize(4,0.0);
  body_Q_[3] = 0.0;

}

ArmControlModule::~ArmControlModule()
{
  queue_thread_.join();
}

void ArmControlModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_sec_ = control_cycle_msec * 0.001;
  queue_thread_      = boost::thread(boost::bind(&ArmControlModule::queueThread, this));

  ros::NodeHandle ros_node;

  // Publisher
  status_msg_pub_       = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
}

void ArmControlModule::queueThread()
{
  ros::NodeHandle     ros_node;
  ros::CallbackQueue  callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  // Subscriber
  ros::Subscriber joint_pose_sub_ = ros_node.subscribe("/robotis/goal_joint_pose", 5,
                                                       &ArmControlModule::goalJointPoseCallback, this);
  ros::Subscriber kinematics_pose_sub_ = ros_node.subscribe("/robotis/goal_kinematics_pose", 5,
                                                            &ArmControlModule::goalKinematicsPoseCallback, this);

  // Service
  ros::ServiceServer get_joint_pose_server = ros_node.advertiseService("/robotis/get_joint_pose",
                                                                       &ArmControlModule::getJointPoseCallback, this);
  ros::ServiceServer get_kinematics_pose_server = ros_node.advertiseService("/robotis/get_kinematics_pose",
                                                                            &ArmControlModule::getKinematicsPoseCallback, this);

  ros::WallDuration duration(control_cycle_sec_);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////callback///////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ArmControlModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status;
  status.header.stamp = ros::Time::now();
  status.type         = type;
  status.module_name  = "ArmControl";
  status.status_msg   = msg;

  status_msg_pub_.publish(status);
}

bool ArmControlModule::getJointPoseCallback(social_robot_arm_sdk::GetJointPose::Request &req,
                                            social_robot_arm_sdk::GetJointPose::Response &res)
{
//  ROS_INFO("getJointPoseCallback");
  for (int i=0; i<number_of_joints_; i++)
  {
    res.pose.pose.name.push_back(joint_id_to_name_[i+1]);
    res.pose.pose.position.push_back(des_joint_pos_[i]);
  }

  return true;
}

bool ArmControlModule::getKinematicsPoseCallback(social_robot_arm_sdk::GetKinematicsPose::Request &req,
                                                social_robot_arm_sdk::GetKinematicsPose::Response &res)
{
  std::string group_name = req.name;

  geometry_msgs::Pose msg;
//  ROS_INFO("getKinematicsPoseCallback");
  if (group_name == "left_arm")
  {
//    ROS_INFO("getKinematicsPoseCallback left_arm");
//    ROS_INFO("larm position x : %f y: %f, z: %f", pre_larm_pos_[0], pre_larm_pos_[1], pre_larm_pos_[2]);
    msg.position.x = pre_larm_pos_[0];
    msg.position.y = pre_larm_pos_[1];
    msg.position.z = pre_larm_pos_[2];

    msg.orientation.x = pre_larm_Q_[0];
    msg.orientation.y = pre_larm_Q_[1];
    msg.orientation.z = pre_larm_Q_[2];
    msg.orientation.w = pre_larm_Q_[3];
//    ROS_INFO("getKinematicsPoseCallback left_arm done");
  }
  else if (group_name == "right_arm")
  {
//    ROS_INFO("getKinematicsPoseCallback right_arm");
//    ROS_INFO("rarm position x : %f y: %f, z: %f", pre_rarm_pos_[0], pre_rarm_pos_[1], pre_rarm_pos_[2]);
    msg.position.x = pre_rarm_pos_[0];
    msg.position.y = pre_rarm_pos_[1];
    msg.position.z = pre_rarm_pos_[2];

    msg.orientation.x = pre_rarm_Q_[0];
    msg.orientation.y = pre_rarm_Q_[1];
    msg.orientation.z = pre_rarm_Q_[2];
    msg.orientation.w = pre_rarm_Q_[3];
//    ROS_INFO("getKinematicsPoseCallback right_arm done");
  }
  else
  {
    ROS_INFO("[getKinematicsPoseCallback] unknown group_name.");
  }

  res.pose.pose = msg;

  return true;
}

void ArmControlModule::goalJointPoseCallback(const social_robot_arm_sdk::JointPose& msg)
{
  if (enable_ == false)
  {
    ROS_ERROR("%s is not enabled!!", module_name_.c_str());
    return;
  }

//  ROS_INFO("goalJointPoseCallback");
  if (control_type_ == NONE || control_type_ == JOINT_CONTROL)
  {
    mov_time_ = msg.mov_time;

    for(int8_t j=0; j<number_of_joints_; j++)
      goal_joint_pos_[j] = des_joint_pos_[j];

    for (size_t i = 0; i < msg.pose.name.size(); i++)
    {
//      for(int8_t j = 0; j < number_of_joints_; j++)
//      {
//        ROS_INFO("%d",joint_name_to_id_[msg.pose.name[i]]);
//        if(joint_id_to_name_[j+1]==msg.pose.name[i])
//        {
//          goal_joint_pos_[joint_name_to_id_[msg.pose.name[i]] - 1] = msg.pose.position[i];
//          ROS_INFO("IN");
//        }
//        else
//        {
//          goal_joint_pos_[j] = des_joint_pos_[j];
//          ROS_INFO("OUT");
//        }
//      }
      std::string joint_name = msg.pose.name[i];
      goal_joint_pos_[joint_name_to_id_[joint_name] - 1] = msg.pose.position[i];
    }

    joint_control_initialize_ = false;
    control_type_ = JOINT_CONTROL;
  }
  else
    ROS_WARN("[WARN] Control type is different!");
}


void ArmControlModule::goalKinematicsPoseCallback(const social_robot_arm_sdk::KinematicsPose& msg)
{
  if (enable_ == false)
  {
    ROS_ERROR("%s is not enabled!!", module_name_.c_str());
    return;
  }

//  ROS_INFO("goalKinematicsPoseCallback");
  if (control_type_ == NONE || control_type_ == TASK_CONTROL)
  {
    if (is_moving_ == true)
    {
      if (task_control_group_!=msg.name)
      {
        ROS_WARN("[WARN] Control group is different!");
        return;
      }
    }

    des_rarm_pos_ = pre_rarm_pos_;
    des_larm_pos_ = pre_larm_pos_;
    des_rarm_Q_ = pre_rarm_Q_;
    des_larm_Q_ = pre_larm_Q_;

    mov_time_ = msg.mov_time;
    task_control_group_ = msg.name;
    task_goal_msg_ = msg.pose;

    task_control_initialize_ = false;
    control_type_ = TASK_CONTROL;
  }
  else
    ROS_WARN("[WARN] Control type is different!");
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////JointControl///////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ArmControlModule::initJointControl()
{
  if (joint_control_initialize_ == true)
    return;

  joint_control_initialize_ = true;

  double ini_time = 0.0;
  double mov_time = mov_time_;

  mov_step_ = 0;
  mov_size_ = (int) (mov_time / control_cycle_sec_) + 1;

  joint_tra_ =
      new robotis_framework::MinimumJerk(ini_time, mov_time,
                                         des_joint_pos_, des_joint_vel_, des_joint_accel_,
                                         goal_joint_pos_, goal_joint_vel_, goal_joint_accel_);
  if (is_moving_ == true)
    ROS_INFO("[UPDATE] Joint Control");
  else
  {
    is_moving_ = true;
    ROS_INFO("[START] Joint Control");
  }
}

void ArmControlModule::calcJointControl()
{
  if (is_moving_ == true)
  {
    double cur_time = (double) mov_step_ * control_cycle_sec_;

    queue_mutex_.lock();

    des_joint_pos_ = joint_tra_->getPosition(cur_time);
    des_joint_vel_ = joint_tra_->getVelocity(cur_time);
    des_joint_accel_ = joint_tra_->getAcceleration(cur_time);

    queue_mutex_.unlock();

    if (mov_step_ == mov_size_-1)
    {
      mov_step_ = 0;
      is_moving_ = false;
      delete joint_tra_;

      control_type_ = NONE;

      ROS_INFO("[END] Joint Control");
    }
    else
      mov_step_++;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////TaskControl////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ArmControlModule::initTaskControl()
{
  if (task_control_initialize_ == true)
    return;

  task_control_initialize_ = true;

  double ini_time = 0.0;
  double mov_time = mov_time_;

  mov_step_ = 0;
  mov_size_ = (int) (mov_time / control_cycle_sec_) + 1;

  task_control_ =
      new TaskControl(task_control_group_,
                      ini_time, mov_time,
                      task_goal_msg_);

  if (is_moving_ == true)
  {
    // TODO
    // ROS_INFO("[UPDATE] Wholebody Control");
    // task_control_->update();
  }
  else
  {
    ROS_INFO("[START] Task Control");

    if(task_control_group_=="left_arm")
    {
      task_control_->initialize(des_larm_pos_, des_larm_Q_);
    }
    if(task_control_group_=="right_arm")
    {
      task_control_->initialize(des_rarm_pos_, des_rarm_Q_);
    }
    is_moving_ = true;
  }
}

void ArmControlModule::calcTaskControl()
{
  if (is_moving_ == true)
  {
    double cur_time = (double) mov_step_ * control_cycle_sec_;

    if(task_control_group_=="left_arm")
    {
      task_control_->set(cur_time);
      task_control_->getTaskPosition(des_larm_pos_);
      task_control_->getTaskOrientation(des_larm_Q_);
    }
    if(task_control_group_=="right_arm")
    {
      task_control_->set(cur_time);
      task_control_->getTaskPosition(des_rarm_pos_);
      task_control_->getTaskOrientation(des_rarm_Q_);
    }

//    ROS_INFO("--");

//    ROS_INFO("pre rarm_pos x: %f, y: %f, z: %f", pre_rarm_pos_[0], pre_rarm_pos_[1], pre_rarm_pos_[2]);
//    ROS_INFO("pre larm_pos x: %f, y: %f, z: %f", pre_larm_pos_[0], pre_larm_pos_[1], pre_larm_pos_[2]);

//    ROS_INFO("des_larm_pos_ x: %f, y: %f, z: %f", des_larm_pos_[0], des_larm_pos_[1], des_larm_pos_[2]);
//    ROS_INFO("des_rarm_pos_ x: %f, y: %f, z: %f", des_rarm_pos_[0], des_rarm_pos_[1], des_rarm_pos_[2]);

    if (mov_step_ == mov_size_-1)
    {
      mov_step_ = 0;
      is_moving_ = false;
      task_control_->finalize();

      control_type_ = NONE;

      ROS_INFO("[END] Task Control");
    }
    else
      mov_step_++;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////RobotPose//////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ArmControlModule::calcRobotPose()
{

  Eigen::MatrixXd body_pos = Eigen::MatrixXd::Zero(3,1);
  body_pos.coeffRef(0,0) = body_pos_[0];
  body_pos.coeffRef(1,0) = body_pos_[1];
  body_pos.coeffRef(2,0) = body_pos_[2];

  Eigen::Quaterniond body_Q(body_Q_[3],body_Q_[0],body_Q_[1],body_Q_[2]);
  Eigen::MatrixXd des_body_rot = robotis_framework::convertQuaternionToRotation(body_Q);

  social_robot_arm_kd_->initialize(body_pos, des_body_rot);

  // Forward Kinematics
  Eigen::VectorXd r_arm_joint_pos, l_arm_joint_pos;

  r_arm_joint_pos.resize(ARM_JOINT_NUM);
  r_arm_joint_pos(0) = des_joint_pos_[joint_name_to_id_["RShoulder_Pitch"]-1];
  r_arm_joint_pos(1) = des_joint_pos_[joint_name_to_id_["RShoulder_Roll"]-1];
  r_arm_joint_pos(2) = des_joint_pos_[joint_name_to_id_["RElbow_Pitch"]-1];
  r_arm_joint_pos(3) = des_joint_pos_[joint_name_to_id_["RElbow_Yaw"]-1];
  r_arm_joint_pos(4) = des_joint_pos_[joint_name_to_id_["RWrist_Pitch"]-1];

  l_arm_joint_pos.resize(ARM_JOINT_NUM);
  l_arm_joint_pos(0) = des_joint_pos_[joint_name_to_id_["LShoulder_Pitch"]-1];
  l_arm_joint_pos(1) = des_joint_pos_[joint_name_to_id_["LShoulder_Roll"]-1];
  l_arm_joint_pos(2) = des_joint_pos_[joint_name_to_id_["LElbow_Pitch"]-1];
  l_arm_joint_pos(3) = des_joint_pos_[joint_name_to_id_["LElbow_Yaw"]-1];
  l_arm_joint_pos(4) = des_joint_pos_[joint_name_to_id_["LWrist_Pitch"]-1];

  social_robot_arm_kd_->setJointPosition(r_arm_joint_pos, l_arm_joint_pos);

  pre_larm_pos_.resize(3,0.0);
  pre_larm_Q_.resize(4,0.0);
  pre_rarm_pos_.resize(3,0.0);
  pre_rarm_Q_.resize(4,0.0);

  social_robot_arm_kd_->solveForwardKinematics(pre_rarm_pos_, pre_rarm_Q_,
                                               pre_larm_pos_, pre_larm_Q_);

  //Task control
  if (control_type_ == TASK_CONTROL)
  {
    std::vector<double_t> rarm_output, larm_output;

    Eigen::MatrixXd des_rarm_pos = Eigen::MatrixXd::Zero(3,1);
    Eigen::MatrixXd des_larm_pos = Eigen::MatrixXd::Zero(3,1);

    for (int i=0; i<3; i++)
    {
      des_rarm_pos.coeffRef(i,0) = des_rarm_pos_[i];
      des_larm_pos.coeffRef(i,0) = des_larm_pos_[i];
    }

    Eigen::Quaterniond des_rarm_Q(des_rarm_Q_[3],des_rarm_Q_[0],des_rarm_Q_[1],des_rarm_Q_[2]);
    Eigen::Quaterniond des_larm_Q(des_larm_Q_[3],des_larm_Q_[0],des_larm_Q_[1],des_larm_Q_[2]);

    bool ik_success;
    ik_success = social_robot_arm_kd_->solveInverseKinematics(rarm_output,
                                                              des_rarm_pos,des_rarm_Q,
                                                              larm_output,
                                                              des_larm_pos,des_larm_Q);

    if (ik_success == true)
    {
      des_joint_pos_[joint_name_to_id_["LShoulder_Pitch"]-1] = larm_output[0];
      des_joint_pos_[joint_name_to_id_["LShoulder_Roll"]-1] = larm_output[1];
      des_joint_pos_[joint_name_to_id_["LElbow_Pitch"]-1] = larm_output[2];
      des_joint_pos_[joint_name_to_id_["LElbow_Yaw"]-1] = larm_output[3];
      des_joint_pos_[joint_name_to_id_["LWrist_Pitch"]-1] = larm_output[4];

      des_joint_pos_[joint_name_to_id_["RShoulder_Pitch"]-1] = rarm_output[0];
      des_joint_pos_[joint_name_to_id_["RShoulder_Roll"]-1] = rarm_output[1];
      des_joint_pos_[joint_name_to_id_["RElbow_Pitch"]-1] = rarm_output[2];
      des_joint_pos_[joint_name_to_id_["RElbow_Yaw"]-1] = rarm_output[3];
      des_joint_pos_[joint_name_to_id_["RWrist_Pitch"]-1] = rarm_output[4];
    }
    else
    {
      mov_step_ = 0;
      is_moving_ = false;
      task_control_->finalize();

      control_type_ = NONE;

      ROS_ERROR("[END] Task Control : IK Failed");
    }
  }

  social_robot_arm_kd_->finalize();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////Process////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ArmControlModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                              std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

  /*----- write curr position -----*/
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;

    robotis_framework::Dynamixel *dxl = NULL;
    std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find(joint_name);
    if (dxl_it != dxls.end())
      dxl = dxl_it->second;
    else
      continue;

    double curr_joint_pos = dxl->dxl_state_->present_position_;
    double goal_joint_pos = dxl->dxl_state_->goal_position_;

    if (goal_initialize_ == false)
      des_joint_pos_[joint_name_to_id_[joint_name]-1] = goal_joint_pos;

    curr_joint_pos_[joint_name_to_id_[joint_name]-1] = curr_joint_pos;
  }

  goal_initialize_ = true;

  /* Trajectory Calculation */
  ros::Time begin = ros::Time::now();
  if (control_type_ == JOINT_CONTROL)
  {
    initJointControl();
    calcJointControl();
  }
  else if (control_type_ == TASK_CONTROL)
  {
    initTaskControl();
    calcTaskControl();
  }

  calcRobotPose();

  sensor_msgs::JointState goal_joint_msg;

  goal_joint_msg.header.stamp = ros::Time::now();
  /*----- set joint data -----*/
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
    result_[joint_name]->goal_position_ = des_joint_pos_[joint_name_to_id_[joint_name]-1];
  }

}

void ArmControlModule::stop()
{
  is_moving_                  = false;
  goal_initialize_            = false;
  joint_control_initialize_   = false;

  control_type_ = NONE;

  return;
}

bool ArmControlModule::isRunning()
{
  return is_moving_;
}
