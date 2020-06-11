/*******************************************************************************
 * Copyright (c) 2018, ROBOTIS CO., LTD.
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

#include "robotis_controller_msgs/SyncWriteItem.h"
#include "social_robot_arm_sdk/arm_motion_control_module.h"

using namespace social_robot_arm;

ArmMotionControlModule::ArmMotionControlModule()
  : control_cycle_msec_(8)
{
  enable_       = false;
  module_name_  = "arm_motion_control_module";
  control_mode_ = robotis_framework::PositionControl;
}

ArmMotionControlModule::~ArmMotionControlModule()
{
  queue_thread_.join();
}

void ArmMotionControlModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&ArmMotionControlModule::queueThread, this));

  robot_ = robot;

  // init result, joint_id_table
  for (auto& _it : robot_->dxls_)
  {
    std::string                   _joint_name = _it.first;
    robotis_framework::Dynamixel* _dxl_info   = _it.second;

    joint_name_to_id_[_joint_name] = _dxl_info->id_;
    result_[_joint_name] = new robotis_framework::DynamixelState();
    result_[_joint_name]->goal_position_ = _dxl_info->dxl_state_->goal_position_;
  }

  motion_state_ = new MotionState(robot_->dxls_.size());
  motion_state_->smp_time_ = 0.001 * control_cycle_msec ;
}

void ArmMotionControlModule::queueThread()
{
  ros::NodeHandle     ros_node;
  ros::CallbackQueue  callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  // subscriber
  play_motion_sub_              = ros_node.subscribe("/arm/play_motion", 10,
                                                     &ArmMotionControlModule::playMotionCallback, this);
  set_joint_delta_position_sub_ = ros_node.subscribe("/arm/set_joint_delta_position", 10,
                                                     &ArmMotionControlModule::setJointDeltaPositionCallback, this);
  set_joint_torque_sub_         = ros_node.subscribe("/arm/set_joint_torque", 10,
                                                     &ArmMotionControlModule::setJointTorqueCallback, this);

  // publisher
  sync_write_item_pub_          = ros_node.advertise<robotis_controller_msgs::SyncWriteItem>("/robotis/sync_write_item", 5);

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void ArmMotionControlModule::generateMotionTrajProc()
{
  for (int id = 1; id <= motion_state_->num_of_joint_; id++)
  {
    // push current goal position to the init value for generating trajectory
    double _ini_value = motion_state_->joint_cur_pose_.coeff(id, 0);
    // std::cout << _ini_value << ", ";
    // push target goal position to the target value for generating trajectory
    double _tar_value = motion_state_->joint_tar_pose_.coeff(id, 0);

    Eigen::MatrixXd _tra;

    if (motion_state_->num_of_via_ == 0)
    {
      _tra = robotis_framework::calcMinimumJerkTra(_ini_value, 0.0, 0.0, _tar_value, 0.0, 0.0,
                                                   motion_state_->smp_time_, motion_state_->mov_time_);
    }
    else
    {
      Eigen::MatrixXd _via_value = motion_state_->joint_via_pose_.col(id);
      Eigen::MatrixXd _d_via_value = motion_state_->joint_via_dpose_.col(id);
      Eigen::MatrixXd _dd_via_value = motion_state_->joint_via_ddpose_.col(id);

      _tra = robotis_framework::calcMinimumJerkTraWithViaPoints(motion_state_->num_of_via_, _ini_value, 0.0, 0.0,
                                                                _via_value, _d_via_value, _dd_via_value, _tar_value, 0.0,
                                                                0.0, motion_state_->smp_time_,
                                                                motion_state_->via_time_,
                                                                motion_state_->mov_time_);
    // std::cout << "index : " << id << std::endl;
    // std::cout << _ini_value << std::endl;
    // std::cout << "------------------------------" << std::endl;
    // std::cout << _via_value << std::endl;
    // std::cout << "------------------------------" << std::endl;
    // std::cout << _tar_value << std::endl;
    // std::cout << "*****************************" << std::endl;
    motion_state_->calc_joint_tra_.block(0, id, motion_state_->all_time_steps_, 1) = _tra;
    }
  }
  // std::cout << std::endl;
  motion_state_->cnt_ = 0;
  motion_state_->is_moving_ = true;

  ROS_INFO("[START] motion trajectory");
}

void ArmMotionControlModule::playMotionCallback(const social_robot_arm_sdk::PlayMotion::ConstPtr &msg)
{
  if (enable_ == false)
  {
    ROS_ERROR("%s is not enabled!!", module_name_.c_str());
    return;
  }
  
  if (motion_state_->is_generating_ == true)
  {
    ROS_ERROR("The previous motion is being generated.");
    return;
  }

  if (motion_state_->is_moving_ == true)
  {
    ROS_ERROR("The previous motion is running.");
    return;
  }

  // msg -> pose
  // check via
  if(msg->pose.size() != msg->delta_time_ms.size() || msg->pose.size() == 0)
    return;

  // movement time
  motion_state_->mov_time_ = accumulate(msg->delta_time_ms.begin(), msg->delta_time_ms.end(), 0);

  // push number of via-point
  int _number_of_via = msg->pose.size() - 1;
  motion_state_->num_of_via_ = _number_of_via;

  // initialize via section
  motion_state_->via_time_.resize(_number_of_via, motion_state_->num_of_joint_ + 1);
  motion_state_->joint_via_pose_.resize(_number_of_via, motion_state_->num_of_joint_ + 1);
  motion_state_->joint_via_dpose_.resize(_number_of_via, motion_state_->num_of_joint_ + 1);
  motion_state_->joint_via_ddpose_.resize(_number_of_via, motion_state_->num_of_joint_ + 1);

  motion_state_->joint_via_pose_.fill(0.0);
  motion_state_->joint_via_dpose_.fill(0.0);
  motion_state_->joint_via_ddpose_.fill(0.0);

  // push via-point to the motion_state
  double via_accum_time = 0.0;
  for (int num = 0; num < _number_of_via; num++)
  {
    sensor_msgs::JointState _joint_pose = msg->pose[num];
    double _via_time = msg->delta_time_ms[num] * 0.001;
    via_accum_time += _via_time;
    motion_state_->via_time_.coeffRef(num, 0) = via_accum_time;

    for(int joint_index = 0; joint_index < _joint_pose.name.size(); joint_index++)
    {
      std::string _joint_name = _joint_pose.name[joint_index];
      int _joint_id = joint_name_to_id_[_joint_name];
      double _value = _joint_pose.position[joint_index];
      double _v_value = 0.0, _a_value = 0.0;
      if(_joint_pose.velocity.size() > joint_index)
        _v_value = _joint_pose.velocity[joint_index];
      if(_joint_pose.effort.size() > joint_index)
        _a_value = _joint_pose.effort[joint_index];

      motion_state_->joint_via_pose_.coeffRef(num, _joint_id) = _value;
      motion_state_->joint_via_dpose_.coeffRef(num, _joint_id) = _v_value;
      motion_state_->joint_via_ddpose_.coeffRef(num, _joint_id) = _a_value;
    }
    // for(int joint_index = 0; joint_index < _joint_pose.name.size(); joint_index++) 
    // {
    //   std::cout << motion_state_->joint_via_pose_.coeffRef(num, joint_index) << " , ";

    // }
    // std::cout << std::endl;
  }

  // push target to the motion_state
  sensor_msgs::JointState _joint_pose = msg->pose[_number_of_via];
  double _via_time = msg->delta_time_ms[_number_of_via] * 0.001;
  via_accum_time += _via_time;
  //motion_state_->via_time_.coeffRef(_number_of_via, 0) = via_accum_time;
  motion_state_->mov_time_ = via_accum_time;

  for(int joint_index = 0; joint_index < _joint_pose.name.size(); joint_index++)
  {
    std::string _joint_name = _joint_pose.name[joint_index];
    int _joint_id = joint_name_to_id_[_joint_name];
    double _value = _joint_pose.position[joint_index];

    motion_state_->joint_tar_pose_.coeffRef(_joint_id, 0) = _value;
  }

  motion_state_->all_time_steps_ = int(motion_state_->mov_time_ / motion_state_->smp_time_) + 1;
  motion_state_->calc_joint_tra_.resize(motion_state_->all_time_steps_, motion_state_->num_of_joint_ + 1);
  // for (int num = 0; num < _number_of_via; num++) {
  //   for(int joint_index = 0; joint_index < _joint_pose.name.size(); joint_index++) 
  //   {
  //     std::cout << motion_state_->joint_via_pose_.coeffRef(num, joint_index) << " , ";

  //   }
  //   std::cout << std::endl;
  // }
  // for(int joint_index = 0; joint_index < _joint_pose.name.size(); joint_index++) 
  // {
  //   std::cout << motion_state_->joint_tar_pose_.coeffRef(joint_index, 0) << " , ";

  // }
    // std::cout << std::endl;
  gen_traj_thread_ = boost::thread(boost::bind(&ArmMotionControlModule::generateMotionTrajProc, this));
}

void ArmMotionControlModule::setJointDeltaPositionCallback(const social_robot_arm_sdk::SetJointDeltaPosition::ConstPtr &msg)
{
  for (int i = 0; i < msg->joint_name.size(); i++)
  {
    result_[msg->joint_name[i]]->goal_position_ += msg->delta_position[i];
  }
}

void ArmMotionControlModule::setJointTorqueCallback(const social_robot_arm_sdk::SetJointTorque::ConstPtr &msg)
{
  robotis_controller_msgs::SyncWriteItem _torque_msg;
  _torque_msg.item_name = "torque_enable";

  for (int i = 0; i < msg->joint_name.size(); i++)
  {
    std::string _joint_name = msg->joint_name[i];
    bool        _torque     = msg->torque[i];

    auto _dxl_it = robot_->dxls_.find(_joint_name);
    if (_dxl_it != robot_->dxls_.end())
    {
      _torque_msg.joint_name.push_back(_joint_name);
      _torque_msg.value.push_back((_torque)? 1 : 0);
    }
  }

  if (_torque_msg.joint_name.size() > 0)
    sync_write_item_pub_.publish(_torque_msg);
}

void ArmMotionControlModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

  // get dxl data : goal position
  for (auto& _it : dxls)
  {
    std::string _joint_name = _it.first;
    robotis_framework::Dynamixel *_dxl = _it.second;

    auto _joint_id_it = joint_name_to_id_.find(_joint_name);
    if (_joint_id_it != joint_name_to_id_.end())
    {
      int _joint_id = _joint_id_it->second;
      motion_state_->joint_cur_pose_.coeffRef(_joint_id, 0) = _dxl->dxl_state_->goal_position_;
    }
  }

  // check data to set
  // if moving
  if (motion_state_->is_moving_ == true)
  {
    //if (motion_state_->cnt_ == 1)
    //  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Init Pose");

    for (auto& _it : result_)
    {
      std::string _joint_name = _it.first;
      int _joint_id = joint_name_to_id_[_joint_name];
      _it.second->goal_position_ = motion_state_->calc_joint_tra_.coeff(motion_state_->cnt_, _joint_id);
    }

    motion_state_->cnt_++;
  }
  else  // if not, send current goal position to the result
  {
    for (auto& _it : result_)
    {
      std::string _joint_name = _it.first;
      int _joint_id = joint_name_to_id_[_joint_name];
      _it.second->goal_position_ = motion_state_->joint_cur_pose_.coeff(_joint_id, 0);
    }
  }

  // initialize count number
  if ((motion_state_->cnt_ >= motion_state_->all_time_steps_) && (motion_state_->is_moving_ == true))
  {
    ROS_INFO("[end] send trajectory");

    //publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Finish Init Pose");

    motion_state_->is_moving_ = false;
    motion_state_->cnt_ = 0;
  }
}

void ArmMotionControlModule::stop()
{
  return;
}

bool ArmMotionControlModule::isRunning()
{
  return false;
}
