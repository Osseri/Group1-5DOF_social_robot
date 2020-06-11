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

/**
 * @file /include/social_robot_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef SOCIAL_ROBOT_GUI_QNODE_HPP_
#define SOCIAL_ROBOT_GUI_QNODE_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#ifndef Q_MOC_RUN

#include <map>
#include <vector>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <string>
#include <boost/thread.hpp>
#include <QThread>
#include <QStringListModel>

#include <Eigen/Dense>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include "social_robot_arm_sdk/JointPose.h"
#include "social_robot_arm_sdk/KinematicsPose.h"

#include "social_robot_arm_sdk/GetKinematicsPose.h"
#include "social_robot_arm_sdk/GetJointPose.h"
#include "social_robot_arm_sdk/PlayMotion.h"

// for test
#include <cstdlib>

#endif

#define DEG2RAD   (M_PI / 180.0)
#define RAD2DEG   (180.0 / M_PI)

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace social_robot_arm_gui
{

/*****************************************************************************
 ** Class
 *****************************************************************************/

class QNode: public QThread
{
  Q_OBJECT
public:
  QNode(int argc, char** argv);
  virtual ~QNode();
  bool init();
  void run();

  /*********************
   ** Logging
   **********************/
  enum LogLevel
  {
    Debug, Info, Warn, Error, Fatal
  };

  QStringListModel* loggingModel()
  {
    return &logging_model_;
  }
  void log(const LogLevel &level, const std::string &msg, std::string sender = "GUI");

  void sendSetModeMsg();

  void sendJointPoseMsg(social_robot_arm_sdk::JointPose msg);
  void sendKinematicsPoseMsg(social_robot_arm_sdk::KinematicsPose msg);

  void presentJointCallback(const sensor_msgs::JointState &msg);
  // motion play module
  void sendSetMode(const std::string &mode_name);
  // void sendPlayMotionMsg(int num_motion, bool go_zero = false);
  void sendPlayMotionMsg(std::vector<std::string> file_times, std::vector<std::string> hw_file_times, std::string file_name);
  void sendPlayMotionMsgHW(std::vector<std::string> file_times);
  void sendHeadWaist(social_robot_arm_sdk::JointPose joint_pos);
  void sendHeadWaistThread(social_robot_arm_sdk::JointPose tar_joints);
  ros::Publisher motion_pub_;

public Q_SLOTS:
  void getJointPose();
  void getKinematicsPose(std::string group_name);

Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();

  void updateCurrentJointPose(social_robot_arm_sdk::JointPose);
  void updateLeftCurrentKinematicsPose(social_robot_arm_sdk::KinematicsPose);
  void updateRightCurrentKinematicsPose(social_robot_arm_sdk::KinematicsPose);

private:
  int     init_argc_;
  char**  init_argv_;

  ros::Publisher      chatter_publisher_;
  QStringListModel    logging_model_;

  ros::Publisher      set_ctrl_module_pub_;
  ros::Publisher      joint_pose_msg_pub_;
  ros::Publisher      head_waist_pose_msg_pub_;
  ros::Publisher      kinematics_pose_msg_pub_;

  ros::Publisher      wr_pos_pub_;
  ros::Publisher      wp_pos_pub_;
  ros::Publisher      hy_pos_pub_;
  ros::Publisher      hp_pos_pub_;

  ros::ServiceClient  get_joint_pose_client_;
  ros::ServiceClient  get_kinematics_pose_client_;

  ros::Subscriber     status_msg_sub_;
  ros::Subscriber     present_joint_msg_sub_;

  std::vector<sensor_msgs::JointState> joint_vector;
  ros::Time tar_time;
  ros::Time last_record_time;
  ros::Time curr_time;
  std::string tar_file;
  sensor_msgs::JointState joint_state;
};

}  // namespace social_robot_gui

#endif /* SOCIAL_ROBOT_GUI_QNODE_HPP_ */
