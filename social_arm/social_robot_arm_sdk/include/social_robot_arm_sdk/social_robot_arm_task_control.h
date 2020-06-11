
#ifndef SOCIAL_ROBOT_ARM_SDK_SOCIAL_ROBOT_ARM_TASK_CONTROL_H_
#define SOCIAL_ROBOT_ARM_SDK_SOCIAL_ROBOT_ARM_TASK_CONTROL_H_

#pragma once

#include <math.h>
#include <stdint.h>
#include <string>
#include <vector>
#include <map>

#include <geometry_msgs/Pose.h>

#include "robotis_math/robotis_math.h"

namespace social_robot_arm
{

class TaskControl
{
public:
  TaskControl(std::string control_group,
                   double init_time, double fin_time,
                   geometry_msgs::Pose goal_msg);
  virtual ~TaskControl();

  void initialize(std::vector<double_t> init_arm_pos, std::vector<double_t> init_arm_Q);
  void update();
  void finalize();

  void set(double time);

  void getTaskPosition(std::vector<double_t> &arm_pos);

  void getTaskVelocity(std::vector<double_t> &arm_vel);
  void getTaskAcceleration(std::vector<double_t> &arm_accel);
  void getTaskOrientation(std::vector<double_t> &arm_Q);

private:
  robotis_framework::MinimumJerk *task_trajectory_;

  std::string control_group_;
  double init_time_, fin_time_;
  geometry_msgs::Pose goal_msg_;

  std::vector<double_t> init_arm_pos_, init_arm_vel_, init_arm_accel_;
  std::vector<double_t> des_arm_pos_, des_arm_vel_, des_arm_accel_;
  std::vector<double_t> goal_arm_pos_, goal_arm_vel_, goal_arm_accel_;
  Eigen::Quaterniond    init_arm_Q_, des_arm_Q_, goal_arm_Q_;

  std::vector<double_t> goal_task_pos_, goal_task_vel_, goal_task_accel_;
  Eigen::Quaterniond    init_task_Q_, des_task_Q_, goal_task_Q_;
};

}

#endif
