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

#ifndef SOCIAL_ROBOT_CONTROL_MODULE_H_
#define SOCIAL_ROBOT_CONTROL_MODULE_H_

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include "robotis_math/robotis_math.h"
#include "robotis_framework_common/motion_module.h"

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/StatusMsg.h"

#include "social_robot_arm_kinematics.h"
#include "social_robot_arm_task_control.h"

#include "social_robot_arm_sdk/JointPose.h"
#include "social_robot_arm_sdk/KinematicsPose.h"

#include "social_robot_arm_sdk/GetJointPose.h"
#include "social_robot_arm_sdk/GetKinematicsPose.h"

namespace social_robot_arm
{

#define NUM_OF_JOINTS (12)

enum CONTROL_TYPE {
  JOINT_CONTROL,
  TASK_CONTROL,
  NONE
};

class ArmControlModule: public robotis_framework::MotionModule,
                        public robotis_framework::Singleton<ArmControlModule>
{
public:
  ArmControlModule();
  virtual ~ArmControlModule();

  /* ROS Topic Callback Functions */
  void goalJointPoseCallback(const social_robot_arm_sdk::JointPose &msg);
  void goalKinematicsPoseCallback(const social_robot_arm_sdk::KinematicsPose& msg);

  /* ROS Service Functions */
  bool getJointPoseCallback(social_robot_arm_sdk::GetJointPose::Request &req,
                            social_robot_arm_sdk::GetJointPose::Response &res);
  bool getKinematicsPoseCallback(social_robot_arm_sdk::GetKinematicsPose::Request &req,
                                 social_robot_arm_sdk::GetKinematicsPose::Response &res);

  /* ROS Framework Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);
  void stop();
  bool isRunning();

  /* ROS Publish Functions */
  void publishStatusMsg(unsigned int type, std::string msg);

  /* Parameter */
  SocialRobotArmKinematics *social_robot_arm_kd_;
  TaskControl  *task_control_;

private:
  void queueThread();

  void initJointControl();
  void calcJointControl();
  void initTaskControl();
  void calcTaskControl();
  void initTaskControl2();
  void calcTaskControl2();

  void calcRobotPose();

  std::map<std::string, int> joint_name_to_id_;
  std::map<int, std::string> joint_id_to_name_;

  double          control_cycle_sec_;
  boost::thread   queue_thread_;
  boost::mutex    queue_mutex_;

  ros::Publisher  status_msg_pub_;

  CONTROL_TYPE control_type_;

  size_t number_of_joints_;
  std::string task_control_group_;

  bool    is_moving_;
  int     mov_size_, mov_step_;
  double  mov_time_;

  bool goal_initialize_;  bool joint_control_initialize_;
  bool task_control_initialize_;

  robotis_framework::MinimumJerk *joint_tra_;

  geometry_msgs::Pose task_goal_msg_;

  // Joint Command
  std::vector<double_t> curr_joint_accel_, curr_joint_vel_, curr_joint_pos_;
  std::vector<double_t> des_joint_accel_,  des_joint_vel_,  des_joint_pos_;
  std::vector<double_t> goal_joint_accel_, goal_joint_vel_, goal_joint_pos_;

  std::vector<double_t> des_larm_pos_, des_larm_vel_, des_larm_accel_, des_larm_Q_;
  std::vector<double_t> des_rarm_pos_, des_rarm_vel_, des_rarm_accel_, des_rarm_Q_;

  std::vector<double_t> pre_larm_pos_, pre_larm_Q_;
  std::vector<double_t> pre_rarm_pos_, pre_rarm_Q_;

  std::vector<double_t> body_pos_, body_Q_;

};

}

#endif /* social_robot */
