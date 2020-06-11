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
 * @file /include/social_robot_gui/main_window.hpp
 *
 * @brief Qt based gui for social_robot_gui.
 *
 * @date November 2010
 **/
#ifndef SOCIAL_ROBOT_GUI_MAIN_WINDOW_H
#define SOCIAL_ROBOT_GUI_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QTimer>

#endif

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace social_robot_arm_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget *parent = 0);
  ~MainWindow();

  void closeEvent(QCloseEvent *event); // Overloaded function

  /******************************************
    ** Transformation
    *******************************************/
  Eigen::MatrixXd rotationX( double angle );
  Eigen::MatrixXd rotationY( double angle );
  Eigen::MatrixXd rotationZ( double angle );

  Eigen::MatrixXd rotation2rpy( Eigen::MatrixXd rotation );
  Eigen::MatrixXd rpy2rotation( double roll, double pitch, double yaw );

  Eigen::Quaterniond rpy2quaternion( double roll, double pitch, double yaw );
  Eigen::Quaterniond rotation2quaternion( Eigen::MatrixXd rotation );

  Eigen::MatrixXd quaternion2rpy( Eigen::Quaterniond quaternion );
  Eigen::MatrixXd quaternion2rotation( Eigen::Quaterniond quaternion );

  void parseIniPoseData(const std::string &path);
  void parseSetPoseData(const std::string &path);
  void init_pose_combo();

public Q_SLOTS:
  /******************************************
   ** Auto-connections (connectSlotsByName())
   *******************************************/
  void on_timer_count();
  void on_actionAbout_triggered();

  void on_set_mode_button_clicked( bool check );

  void on_joint_send_box_clicked( bool check );
  void on_joint_get_box_clicked( bool check );

  void on_task_send_button_clicked( bool check );
//  void on_task_get_button_clicked( bool check );
  void on_task_lfinger_send_box_clicked( bool check );
  void on_task_rfinger_send_box_clicked( bool check );
  void on_task_left_set_goal_button_clicked( bool check );
  void on_task_right_set_goal_button_clicked( bool check );

  void on_lfinger_grip_on_send_box_clicked( bool check );
  void on_lfinger_grip_off_send_box_clicked( bool check );
  void on_rfinger_grip_on_send_box_clicked( bool check );
  void on_rfinger_grip_off_send_box_clicked( bool check );

  void on_task_left_px_send_box_clicked(bool check);
  void on_task_left_py_send_box_clicked(bool check);
  void on_task_left_pz_send_box_clicked(bool check);
  void on_task_left_nx_send_box_clicked(bool check);
  void on_task_left_ny_send_box_clicked(bool check);
  void on_task_left_nz_send_box_clicked(bool check);

  void on_task_right_px_send_box_clicked(bool check);
  void on_task_right_py_send_box_clicked(bool check);
  void on_task_right_pz_send_box_clicked(bool check);
  void on_task_right_nx_send_box_clicked(bool check);
  void on_task_right_ny_send_box_clicked(bool check);
  void on_task_right_nz_send_box_clicked(bool check);

  void on_init_pose_button_clicked( bool check );
  void on_zero_pose_button_clicked( bool check );

  void on_button_set_motion_clicked(bool check);

  void on_pos_data_load_clicked( bool check );
  void on_pos_data_save_clicked( bool check );
  void on_motion_add_file_clicked(bool check);
  void on_motion_remove_file_clicked(bool check);
  void on_motion_file_play_test_clicked(bool check);
  void on_motion_file_play_save_clicked(bool check);
  

  void on_hw_pos_data_load_clicked( bool check );
  void on_hw_pos_data_save_clicked( bool check );
  void on_hw_motion_add_file_clicked(bool check);
  void on_hw_motion_remove_file_clicked(bool check);

  /******************************************
   ** Manual connections
   *******************************************/
  void updateLoggingView(); // no idea why this can't connect automatically

  void updateCurrJointPoseSpinbox( social_robot_arm_sdk::JointPose msg );
  void updateLeftCurrKinematicsPoseSpinbox( social_robot_arm_sdk::KinematicsPose msg );
  void updateRightCurrKinematicsPoseSpinbox( social_robot_arm_sdk::KinematicsPose msg );

private:
  Ui::MainWindowDesign ui;
  QNode qnode;
  QTimer* ui_timer;

  std::vector<std::string> joint_name;
  QList<QAbstractSpinBox *> joint_box;
  QList<QAbstractSpinBox *> cr_joint_box;
};

}  // namespace social_robot_gui

#endif // SOCIAL_ROBOT_GUI_MAIN_WINDOW_H
