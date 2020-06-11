/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <sstream>
#include <fstream>
#include "social_robot_arm_gui/main_window.hpp"
#include <boost/filesystem.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace social_robot_arm_gui {

using namespace Qt;
using namespace boost::filesystem;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent)
  , qnode(argc,argv)
{
  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

  setWindowIcon(QIcon(":/images/icon.png"));
  ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  joint_name.push_back("LShoulder_Pitch");
  joint_name.push_back("LShoulder_Roll");
  joint_name.push_back("LElbow_Pitch");
  joint_name.push_back("LElbow_Yaw");
  joint_name.push_back("LWrist_Pitch");
  joint_name.push_back("LFinger");

  joint_name.push_back("RShoulder_Pitch");
  joint_name.push_back("RShoulder_Roll");
  joint_name.push_back("RElbow_Pitch");
  joint_name.push_back("RElbow_Yaw");
  joint_name.push_back("RWrist_Pitch");
  joint_name.push_back("RFinger");

  /*********************
    ** Logging
    **********************/
  ui.view_logging->setModel(qnode.loggingModel());
  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

  cr_joint_box.append( ui.cr_l1_box );
  cr_joint_box.append( ui.cr_l2_box );
  cr_joint_box.append( ui.cr_l3_box );
  cr_joint_box.append( ui.cr_l4_box );
  cr_joint_box.append( ui.cr_l5_box );
  cr_joint_box.append( ui.cr_lf_box );
  cr_joint_box.append( ui.cr_r1_box );
  cr_joint_box.append( ui.cr_r2_box );
  cr_joint_box.append( ui.cr_r3_box );
  cr_joint_box.append( ui.cr_r4_box );
  cr_joint_box.append( ui.cr_r5_box );
  cr_joint_box.append( ui.cr_rf_box );

  joint_box.append( ui.l1_box );
  joint_box.append( ui.l2_box );
  joint_box.append( ui.l3_box );
  joint_box.append( ui.l4_box );
  joint_box.append( ui.l5_box );
  joint_box.append( ui.lf_box );

  joint_box.append( ui.r1_box );
  joint_box.append( ui.r2_box );
  joint_box.append( ui.r3_box );
  joint_box.append( ui.r4_box );
  joint_box.append( ui.r5_box );
  joint_box.append( ui.rf_box );

  /****************************
    ** Connect
    ****************************/

  qRegisterMetaType<social_robot_arm_sdk::JointPose>("social_robot_arm_sdk::JointPose");
  QObject::connect(&qnode, SIGNAL(updateCurrentJointPose(social_robot_arm_sdk::JointPose)), this, SLOT(updateCurrJointPoseSpinbox(social_robot_arm_sdk::JointPose)));

  qRegisterMetaType<social_robot_arm_sdk::KinematicsPose>("social_robot_arm_sdk::KinematicsPose");
  QObject::connect(&qnode, SIGNAL(updateLeftCurrentKinematicsPose(social_robot_arm_sdk::KinematicsPose)), this, SLOT(updateLeftCurrKinematicsPoseSpinbox(social_robot_arm_sdk::KinematicsPose)));
  QObject::connect(&qnode, SIGNAL(updateRightCurrentKinematicsPose(social_robot_arm_sdk::KinematicsPose)), this, SLOT(updateRightCurrKinematicsPoseSpinbox(social_robot_arm_sdk::KinematicsPose)));

  ui_timer = new QTimer;
  QObject::connect(ui_timer, SIGNAL(timeout()), this, SLOT(on_timer_count()));

  init_pose_combo();

  /*********************
    ** Auto Start
    **********************/
  qnode.init();
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::on_set_mode_button_clicked( bool check )
{
  qnode.sendSetModeMsg();
  ui_timer->start(10);
}

void MainWindow::on_timer_count()
{
  qnode.getJointPose();
  qnode.getKinematicsPose("left_arm");
  qnode.getKinematicsPose("right_arm");
}

void MainWindow::on_joint_get_box_clicked( bool check )
{
  ui.l1_box->setValue( ui.cr_l1_box->value() );
  ui.l2_box->setValue( ui.cr_l2_box->value() );
  ui.l3_box->setValue( ui.cr_l3_box->value() );
  ui.l4_box->setValue( ui.cr_l4_box->value() );
  ui.l5_box->setValue( ui.cr_l5_box->value() );
  ui.lf_box->setValue( ui.cr_lf_box->value() );

  ui.r1_box->setValue( ui.cr_r1_box->value() );
  ui.r2_box->setValue( ui.cr_r2_box->value() );
  ui.r3_box->setValue( ui.cr_r3_box->value() );
  ui.r4_box->setValue( ui.cr_r4_box->value() );
  ui.r5_box->setValue( ui.cr_r5_box->value() );
  ui.rf_box->setValue( ui.cr_rf_box->value() );
}

void MainWindow::on_task_lfinger_send_box_clicked( bool check )
{
  social_robot_arm_sdk::JointPose msg;
  msg.mov_time = ui.task_mov_time_box->value();

  msg.pose.name.push_back( joint_name[5] );
  msg.pose.position.push_back( ui.task_lf_box->value() * M_PI / 180.0 );

  qnode.sendJointPoseMsg( msg );
}


void MainWindow::on_task_rfinger_send_box_clicked( bool check )
{
  social_robot_arm_sdk::JointPose msg;
  msg.mov_time = ui.task_mov_time_box->value();

  msg.pose.name.push_back( joint_name[11] );
  msg.pose.position.push_back( ui.task_rf_box->value() * M_PI / 180.0 );

  qnode.sendJointPoseMsg( msg );
}


void MainWindow::on_task_left_px_send_box_clicked(bool check)
{
  social_robot_arm_sdk::KinematicsPose msg;

  msg.name = "left_arm";

  msg.mov_time = 0.5;
  msg.pose.position.x = ui.left_present_x_box->value();
  msg.pose.position.y = ui.left_present_y_box->value();
  msg.pose.position.z = ui.left_present_z_box->value();

  double roll = ui.left_present_roll_box->value() * DEG2RAD;
  double pitch = ui.left_present_pitch_box->value() * DEG2RAD;
  double yaw = ui.left_present_yaw_box->value() * DEG2RAD;

  Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

  msg.pose.orientation.x = QR.x();
  msg.pose.orientation.y = QR.y();
  msg.pose.orientation.z = QR.z();
  msg.pose.orientation.w = QR.w();

  /////////////////////set move/////////////////
  msg.pose.position.x = msg.pose.position.x + ui.task_move_tick_box->value();

  qnode.sendKinematicsPoseMsg( msg );
}
void MainWindow::on_task_left_py_send_box_clicked(bool check)
{
  social_robot_arm_sdk::KinematicsPose msg;

  msg.name = "left_arm";

  msg.mov_time = 0.5;
  msg.pose.position.x = ui.left_present_x_box->value();
  msg.pose.position.y = ui.left_present_y_box->value();
  msg.pose.position.z = ui.left_present_z_box->value();

  double roll = ui.left_present_roll_box->value() * DEG2RAD;
  double pitch = ui.left_present_pitch_box->value() * DEG2RAD;
  double yaw = ui.left_present_yaw_box->value() * DEG2RAD;

  Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

  msg.pose.orientation.x = QR.x();
  msg.pose.orientation.y = QR.y();
  msg.pose.orientation.z = QR.z();
  msg.pose.orientation.w = QR.w();

  /////////////////////set move/////////////////
  msg.pose.position.y = msg.pose.position.y + ui.task_move_tick_box->value();

  qnode.sendKinematicsPoseMsg( msg );
}
void MainWindow::on_task_left_pz_send_box_clicked(bool check)
{
  social_robot_arm_sdk::KinematicsPose msg;

  msg.name = "left_arm";

  msg.mov_time = 0.5;
  msg.pose.position.x = ui.left_present_x_box->value();
  msg.pose.position.y = ui.left_present_y_box->value();
  msg.pose.position.z = ui.left_present_z_box->value();

  double roll = ui.left_present_roll_box->value() * DEG2RAD;
  double pitch = ui.left_present_pitch_box->value() * DEG2RAD;
  double yaw = ui.left_present_yaw_box->value() * DEG2RAD;

  Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

  msg.pose.orientation.x = QR.x();
  msg.pose.orientation.y = QR.y();
  msg.pose.orientation.z = QR.z();
  msg.pose.orientation.w = QR.w();

  /////////////////////set move/////////////////
  msg.pose.position.z = msg.pose.position.z + ui.task_move_tick_box->value();

  qnode.sendKinematicsPoseMsg( msg );
}
void MainWindow::on_task_left_nx_send_box_clicked(bool check)
{
  social_robot_arm_sdk::KinematicsPose msg;

  msg.name = "left_arm";

  msg.mov_time = 0.5;
  msg.pose.position.x = ui.left_present_x_box->value();
  msg.pose.position.y = ui.left_present_y_box->value();
  msg.pose.position.z = ui.left_present_z_box->value();

  double roll = ui.left_present_roll_box->value() * DEG2RAD;
  double pitch = ui.left_present_pitch_box->value() * DEG2RAD;
  double yaw = ui.left_present_yaw_box->value() * DEG2RAD;

  Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

  msg.pose.orientation.x = QR.x();
  msg.pose.orientation.y = QR.y();
  msg.pose.orientation.z = QR.z();
  msg.pose.orientation.w = QR.w();

  /////////////////////set move/////////////////
  msg.pose.position.x = msg.pose.position.x - ui.task_move_tick_box->value();

  qnode.sendKinematicsPoseMsg( msg );
}
void MainWindow::on_task_left_ny_send_box_clicked(bool check)
{
  social_robot_arm_sdk::KinematicsPose msg;

  msg.name = "left_arm";

  msg.mov_time = 0.5;
  msg.pose.position.x = ui.left_present_x_box->value();
  msg.pose.position.y = ui.left_present_y_box->value();
  msg.pose.position.z = ui.left_present_z_box->value();

  double roll = ui.left_present_roll_box->value() * DEG2RAD;
  double pitch = ui.left_present_pitch_box->value() * DEG2RAD;
  double yaw = ui.left_present_yaw_box->value() * DEG2RAD;

  Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

  msg.pose.orientation.x = QR.x();
  msg.pose.orientation.y = QR.y();
  msg.pose.orientation.z = QR.z();
  msg.pose.orientation.w = QR.w();

  /////////////////////set move/////////////////
  msg.pose.position.y = msg.pose.position.y - ui.task_move_tick_box->value();

  qnode.sendKinematicsPoseMsg( msg );
}
void MainWindow::on_task_left_nz_send_box_clicked(bool check)
{
  social_robot_arm_sdk::KinematicsPose msg;

  msg.name = "left_arm";

  msg.mov_time = 0.5;
  msg.pose.position.x = ui.left_present_x_box->value();
  msg.pose.position.y = ui.left_present_y_box->value();
  msg.pose.position.z = ui.left_present_z_box->value();

  double roll = ui.left_present_roll_box->value() * DEG2RAD;
  double pitch = ui.left_present_pitch_box->value() * DEG2RAD;
  double yaw = ui.left_present_yaw_box->value() * DEG2RAD;

  Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

  msg.pose.orientation.x = QR.x();
  msg.pose.orientation.y = QR.y();
  msg.pose.orientation.z = QR.z();
  msg.pose.orientation.w = QR.w();

  /////////////////////set move/////////////////
  msg.pose.position.z = msg.pose.position.z - ui.task_move_tick_box->value();

  qnode.sendKinematicsPoseMsg( msg );
}

void MainWindow::on_task_right_px_send_box_clicked(bool check)
{
  social_robot_arm_sdk::KinematicsPose msg;

  msg.name = "right_arm";

  msg.mov_time = 0.5;
  msg.pose.position.x = ui.left_present_x_box->value();
  msg.pose.position.y = ui.left_present_y_box->value();
  msg.pose.position.z = ui.left_present_z_box->value();

  double roll = ui.left_present_roll_box->value() * DEG2RAD;
  double pitch = ui.left_present_pitch_box->value() * DEG2RAD;
  double yaw = ui.left_present_yaw_box->value() * DEG2RAD;

  Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

  msg.pose.orientation.x = QR.x();
  msg.pose.orientation.y = QR.y();
  msg.pose.orientation.z = QR.z();
  msg.pose.orientation.w = QR.w();

  /////////////////////set move/////////////////
  msg.pose.position.x = msg.pose.position.x + ui.task_move_tick_box->value();

  qnode.sendKinematicsPoseMsg( msg );
}
void MainWindow::on_task_right_py_send_box_clicked(bool check)
{
  social_robot_arm_sdk::KinematicsPose msg;

  msg.name = "right_arm";

  msg.mov_time = 0.5;
  msg.pose.position.x = ui.left_present_x_box->value();
  msg.pose.position.y = ui.left_present_y_box->value();
  msg.pose.position.z = ui.left_present_z_box->value();

  double roll = ui.left_present_roll_box->value() * DEG2RAD;
  double pitch = ui.left_present_pitch_box->value() * DEG2RAD;
  double yaw = ui.left_present_yaw_box->value() * DEG2RAD;

  Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

  msg.pose.orientation.x = QR.x();
  msg.pose.orientation.y = QR.y();
  msg.pose.orientation.z = QR.z();
  msg.pose.orientation.w = QR.w();

  /////////////////////set move/////////////////
  msg.pose.position.y = msg.pose.position.y + ui.task_move_tick_box->value();

  qnode.sendKinematicsPoseMsg( msg );
}
void MainWindow::on_task_right_pz_send_box_clicked(bool check)
{
  social_robot_arm_sdk::KinematicsPose msg;

  msg.name = "right_arm";

  msg.mov_time = 0.5;
  msg.pose.position.x = ui.left_present_x_box->value();
  msg.pose.position.y = ui.left_present_y_box->value();
  msg.pose.position.z = ui.left_present_z_box->value();

  double roll = ui.left_present_roll_box->value() * DEG2RAD;
  double pitch = ui.left_present_pitch_box->value() * DEG2RAD;
  double yaw = ui.left_present_yaw_box->value() * DEG2RAD;

  Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

  msg.pose.orientation.x = QR.x();
  msg.pose.orientation.y = QR.y();
  msg.pose.orientation.z = QR.z();
  msg.pose.orientation.w = QR.w();

  /////////////////////set move/////////////////
  msg.pose.position.z = msg.pose.position.z + ui.task_move_tick_box->value();

  qnode.sendKinematicsPoseMsg( msg );
}
void MainWindow::on_task_right_nx_send_box_clicked(bool check)
{
  social_robot_arm_sdk::KinematicsPose msg;

  msg.name = "right_arm";

  msg.mov_time = 0.5;
  msg.pose.position.x = ui.left_present_x_box->value();
  msg.pose.position.y = ui.left_present_y_box->value();
  msg.pose.position.z = ui.left_present_z_box->value();

  double roll = ui.left_present_roll_box->value() * DEG2RAD;
  double pitch = ui.left_present_pitch_box->value() * DEG2RAD;
  double yaw = ui.left_present_yaw_box->value() * DEG2RAD;

  Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

  msg.pose.orientation.x = QR.x();
  msg.pose.orientation.y = QR.y();
  msg.pose.orientation.z = QR.z();
  msg.pose.orientation.w = QR.w();

  /////////////////////set move/////////////////
  msg.pose.position.x = msg.pose.position.x - ui.task_move_tick_box->value();

  qnode.sendKinematicsPoseMsg( msg );
}
void MainWindow::on_task_right_ny_send_box_clicked(bool check)
{
  social_robot_arm_sdk::KinematicsPose msg;

  msg.name = "right_arm";

  msg.mov_time = 0.5;
  msg.pose.position.x = ui.left_present_x_box->value();
  msg.pose.position.y = ui.left_present_y_box->value();
  msg.pose.position.z = ui.left_present_z_box->value();

  double roll = ui.left_present_roll_box->value() * DEG2RAD;
  double pitch = ui.left_present_pitch_box->value() * DEG2RAD;
  double yaw = ui.left_present_yaw_box->value() * DEG2RAD;

  Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

  msg.pose.orientation.x = QR.x();
  msg.pose.orientation.y = QR.y();
  msg.pose.orientation.z = QR.z();
  msg.pose.orientation.w = QR.w();

  /////////////////////set move/////////////////
  msg.pose.position.y = msg.pose.position.y - ui.task_move_tick_box->value();

  qnode.sendKinematicsPoseMsg( msg );
}
void MainWindow::on_task_right_nz_send_box_clicked(bool check)
{
  social_robot_arm_sdk::KinematicsPose msg;

  msg.name = "right_arm";

  msg.mov_time = 0.5;
  msg.pose.position.x = ui.left_present_x_box->value();
  msg.pose.position.y = ui.left_present_y_box->value();
  msg.pose.position.z = ui.left_present_z_box->value();

  double roll = ui.left_present_roll_box->value() * DEG2RAD;
  double pitch = ui.left_present_pitch_box->value() * DEG2RAD;
  double yaw = ui.left_present_yaw_box->value() * DEG2RAD;

  Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

  msg.pose.orientation.x = QR.x();
  msg.pose.orientation.y = QR.y();
  msg.pose.orientation.z = QR.z();
  msg.pose.orientation.w = QR.w();

  /////////////////////set move/////////////////
  msg.pose.position.z = msg.pose.position.z - ui.task_move_tick_box->value();

  qnode.sendKinematicsPoseMsg( msg );
}

void MainWindow::on_lfinger_grip_on_send_box_clicked( bool check )
{
  social_robot_arm_sdk::JointPose msg;
  msg.mov_time = 3.0;

  msg.pose.name.push_back( joint_name[5] );
  msg.pose.position.push_back( -130 * M_PI / 180.0 );

  qnode.sendJointPoseMsg( msg );
}

void MainWindow::on_lfinger_grip_off_send_box_clicked( bool check )
{
  social_robot_arm_sdk::JointPose msg;
  msg.mov_time = 3.0;

  msg.pose.name.push_back( joint_name[5] );
  msg.pose.position.push_back( 0 * M_PI / 180.0 );

  qnode.sendJointPoseMsg( msg );
}

void MainWindow::on_rfinger_grip_on_send_box_clicked( bool check )
{
  social_robot_arm_sdk::JointPose msg;
  msg.mov_time = 3.0;

  msg.pose.name.push_back( joint_name[11] );
  msg.pose.position.push_back( 130 * M_PI / 180.0 );

  qnode.sendJointPoseMsg( msg );
}

void MainWindow::on_rfinger_grip_off_send_box_clicked( bool check )
{
  social_robot_arm_sdk::JointPose msg;
  msg.mov_time = 3.0;

  msg.pose.name.push_back( joint_name[11] );
  msg.pose.position.push_back( 0 * M_PI / 180.0 );

  qnode.sendJointPoseMsg( msg );
}


void MainWindow::on_joint_send_box_clicked( bool check )
{
  social_robot_arm_sdk::JointPose msg;
  msg.mov_time = ui.joint_mov_time_box->value();

  //  for ( int _id = 0; _id < joint_box.size(); _id++ )
  //  {
  //    msg.pose.name.push_back( joint_name[ _id ] );
  //    msg.pose.position.push_back( ((QDoubleSpinBox *) joint_box[ _id ])->value() * M_PI / 180.0 );
  //  }
  for ( int _id = 0; _id < joint_box.size(); _id++ )
  {
    msg.pose.name.push_back( joint_name[ _id ] );
    msg.pose.position.push_back( ((QDoubleSpinBox *) joint_box[ _id ])->value() * M_PI / 180.0 );
  }

  qnode.sendJointPoseMsg( msg );

  
  social_robot_arm_sdk::JointPose msg2;
  msg2.mov_time = ui.joint_mov_time_box->value()*1000;
  msg2.pose.name.push_back("Waist_Roll");
  msg2.pose.position.push_back(ui.wr_box->value()* DEG2RAD);
  msg2.pose.name.push_back("Waist_Pitch");
  msg2.pose.position.push_back(ui.wp_box->value()* DEG2RAD);
  msg2.pose.name.push_back("Head_Yaw");
  msg2.pose.position.push_back(ui.hy_box->value()* DEG2RAD);
  msg2.pose.name.push_back("Head_Pitch");
  msg2.pose.position.push_back(ui.hp_box->value()* DEG2RAD);
  qnode.sendHeadWaistThread( msg2 );
}

void MainWindow::on_pos_data_load_clicked( bool check )
{

  QString fileqs = ui.pos_data_combo->currentText();
  std::string pose_path = ros::package::getPath("social_robot_arm_gui") + "/arm/"+fileqs.toStdString();
  parseSetPoseData(pose_path);
  init_pose_combo();
}

void MainWindow::on_hw_pos_data_load_clicked( bool check )
{

  QString fileqs = ui.hw_pos_data_combo->currentText();
  std::string pose_path = ros::package::getPath("social_robot_arm_gui") + "/head_waist/"+fileqs.toStdString();
  parseSetPoseData(pose_path);
  init_pose_combo();
}

void MainWindow::init_pose_combo() {

  std::string path = ros::package::getPath("social_robot_arm_gui") + "/arm";
  boost::filesystem::path p(path);
  for (auto i = directory_iterator(p); i != directory_iterator(); i++)
  {
    if (!is_directory(i->path())) //we eliminate directories
    {
        // std::cout << i->path().filename().string() << endl;
        std::string file_name = i->path().filename().string();
        QString qstr = QString::fromStdString(file_name);
        if(ui.pos_data_combo->findText(qstr) == -1) {
          ui.pos_data_combo->addItem(qstr);
          ui.motion_add_file_combo->addItem(qstr);
        }
        // QString qs = file_name;
    }
    else
        continue;
  }

  std::string hw_path = ros::package::getPath("social_robot_arm_gui") + "/head_waist";
  boost::filesystem::path hw_p(hw_path);
  for (auto i = directory_iterator(hw_p); i != directory_iterator(); i++)
  {
    if (!is_directory(i->path())) //we eliminate directories
    {
        // std::cout << i->path().filename().string() << endl;
        std::string file_name = i->path().filename().string();
        QString qstr = QString::fromStdString(file_name);
        if(ui.hw_pos_data_combo->findText(qstr) == -1) {
          ui.hw_pos_data_combo->addItem(qstr);
          ui.hw_motion_add_file_combo->addItem(qstr);
        }
        // QString qs = file_name;
    }
    else
        continue;
  }
  
}

void MainWindow::on_init_pose_button_clicked( bool check )
{
  std::string ini_pose_path = ros::package::getPath("social_robot_arm_gui") + "/config/init_pose.yaml";
  parseIniPoseData(ini_pose_path);

  std_msgs::Bool msg;
  msg.data = true;
}

void MainWindow::on_zero_pose_button_clicked( bool check )
{
  std::string ini_pose_path = ros::package::getPath("social_robot_arm_gui") + "/config/zero_pose.yaml";
  parseIniPoseData(ini_pose_path);
}

void MainWindow::on_task_send_button_clicked( bool check )
{
  social_robot_arm_sdk::KinematicsPose msg;

  msg.name = ui.task_group_name_box->currentText().toStdString();

  msg.mov_time = ui.task_mov_time_box->value();
  double roll;
  double pitch;
  double yaw;

  if(msg.name=="left_arm")
  {
    msg.pose.position.x = ui.left_goal_x_box->value();
    msg.pose.position.y = ui.left_goal_y_box->value();
    msg.pose.position.z = ui.left_goal_z_box->value();

    roll = ui.left_goal_roll_box->value() * DEG2RAD;
    pitch = ui.left_goal_pitch_box->value() * DEG2RAD;
    yaw = ui.left_goal_yaw_box->value() * DEG2RAD;
  }
  else if(msg.name=="right_arm")
  {
    msg.pose.position.x = ui.right_goal_x_box->value();
    msg.pose.position.y = ui.right_goal_y_box->value();
    msg.pose.position.z = ui.right_goal_z_box->value();

    roll = ui.right_goal_roll_box->value() * DEG2RAD;
    pitch = ui.right_goal_pitch_box->value() * DEG2RAD;
    yaw = ui.right_goal_yaw_box->value() * DEG2RAD;
  }

  Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

  msg.pose.orientation.x = QR.x();
  msg.pose.orientation.y = QR.y();
  msg.pose.orientation.z = QR.z();
  msg.pose.orientation.w = QR.w();

  qnode.sendKinematicsPoseMsg( msg );
}

//void MainWindow::on_task_get_button_clicked( bool check )
//{
//  qnode.getKinematicsPose("left_arm");
//  qnode.getKinematicsPose("right_arm");
//}

void MainWindow::on_task_left_set_goal_button_clicked( bool check )
{
  ui.left_goal_x_box->setValue( ui.left_present_x_box->value() );
  ui.left_goal_y_box->setValue( ui.left_present_y_box->value() );
  ui.left_goal_z_box->setValue( ui.left_present_z_box->value() );
  ui.left_goal_roll_box->setValue( ui.left_present_roll_box->value() );
  ui.left_goal_pitch_box->setValue( ui.left_present_pitch_box->value() );
  ui.left_goal_yaw_box->setValue( ui.left_present_yaw_box->value() );
}

void MainWindow::on_task_right_set_goal_button_clicked( bool check )
{
  ui.right_goal_x_box->setValue( ui.right_present_x_box->value() );
  ui.right_goal_y_box->setValue( ui.right_present_y_box->value() );
  ui.right_goal_z_box->setValue( ui.right_present_z_box->value() );
  ui.right_goal_roll_box->setValue( ui.right_present_roll_box->value() );
  ui.right_goal_pitch_box->setValue( ui.right_present_pitch_box->value() );
  ui.right_goal_yaw_box->setValue( ui.right_present_yaw_box->value() );
}

void MainWindow::updateCurrJointPoseSpinbox( social_robot_arm_sdk::JointPose msg )
{
  // std::cout << "updateCurrJointPoseSpinbox" << std::endl;
  for ( int i=0; i<msg.pose.name.size(); i++ ) {
    // ((QDoubleSpinBox *) cr_joint_box[i])->setValue( msg.pose.position[i] * RAD2DEG );
    std::string joint_name = msg.pose.name[i];
    double value = msg.pose.position[i] * RAD2DEG;
    if(joint_name == "LShoulder_Pitch") {
      ui.cr_l1_box->setValue( value );
    } else if(joint_name == "LShoulder_Roll") {
      ui.cr_l2_box->setValue( value );
    } else if(joint_name == "LElbow_Pitch") {
      ui.cr_l3_box->setValue( value );
    } else if(joint_name == "LElbow_Yaw") {
      ui.cr_l4_box->setValue( value );
    } else if(joint_name == "LWrist_Pitch") {
      ui.cr_l5_box->setValue( value );
    } else if(joint_name == "LFinger") {
      ui.cr_lf_box->setValue( value );
    } else if(joint_name == "RShoulder_Pitch") {
      ui.cr_r1_box->setValue( value );
    } else if(joint_name == "RShoulder_Roll") {
      ui.cr_r2_box->setValue( value );
    } else if(joint_name == "RElbow_Pitch") {
      ui.cr_r3_box->setValue( value );
    } else if(joint_name == "RElbow_Yaw") {
      ui.cr_r4_box->setValue( value );
    } else if(joint_name == "RWrist_Pitch") {
      ui.cr_r5_box->setValue( value );
    } else if(joint_name == "RFinger") {
      ui.cr_rf_box->setValue( value );
    }
    else if(joint_name == "Waist_Roll") {
      ui.cr_wr_box->setValue( value );
    } else if(joint_name == "Waist_Pitch") {
      ui.cr_wp_box->setValue( value );
    } else if(joint_name == "Head_Yaw") {
      ui.cr_hy_box->setValue( value );
    } else if(joint_name == "Head_Pitch") {
      ui.cr_hp_box->setValue( value );
    }

  }
}

void MainWindow::updateLeftCurrKinematicsPoseSpinbox( social_robot_arm_sdk::KinematicsPose msg )
{
  ui.left_present_x_box->setValue( msg.pose.position.x );
  ui.left_present_y_box->setValue( msg.pose.position.y );
  ui.left_present_z_box->setValue( msg.pose.position.z );

  Eigen::Quaterniond QR( msg.pose.orientation.w , msg.pose.orientation.x , msg.pose.orientation.y , msg.pose.orientation.z );
  Eigen::MatrixXd rpy = quaternion2rpy( QR );

  double roll = rpy.coeff( 0 , 0 ) * 180.0 / M_PI;
  double pitch = rpy.coeff( 1 , 0 ) * 180.0 / M_PI;
  double yaw = rpy.coeff( 2, 0 ) * 180.0 /M_PI;

  ui.left_present_roll_box->setValue( roll );
  ui.left_present_pitch_box->setValue( pitch );
  ui.left_present_yaw_box->setValue( yaw );
}

void MainWindow::updateRightCurrKinematicsPoseSpinbox( social_robot_arm_sdk::KinematicsPose msg )
{
  ui.right_present_x_box->setValue( msg.pose.position.x );
  ui.right_present_y_box->setValue( msg.pose.position.y );
  ui.right_present_z_box->setValue( msg.pose.position.z );

  Eigen::Quaterniond QR( msg.pose.orientation.w , msg.pose.orientation.x , msg.pose.orientation.y , msg.pose.orientation.z );
  Eigen::MatrixXd rpy = quaternion2rpy( QR );

  double roll = rpy.coeff( 0 , 0 ) * 180.0 / M_PI;
  double pitch = rpy.coeff( 1 , 0 ) * 180.0 / M_PI;
  double yaw = rpy.coeff( 2, 0 ) * 180.0 /M_PI;

  ui.right_present_roll_box->setValue( roll );
  ui.right_present_pitch_box->setValue( pitch );
  ui.right_present_yaw_box->setValue( yaw );
}


Eigen::MatrixXd MainWindow::rotationX( double angle )
{
  Eigen::MatrixXd rotation(3,3);

  rotation << 1.0, 0.0, 0.0,
      0.0, cos(angle), -sin(angle),
      0.0, sin(angle),  cos(angle);

  return rotation;
}

Eigen::MatrixXd MainWindow::rotationY( double angle )
{
  Eigen::MatrixXd rotation(3,3);

  rotation << cos(angle), 0.0, sin(angle),
      0.0, 1.0, 0.0,
      -sin(angle), 0.0, cos(angle);

  return rotation;
}

Eigen::MatrixXd MainWindow::rotationZ( double angle )
{
  Eigen::MatrixXd rotation(3,3);

  rotation << cos(angle), -sin(angle), 0.0,
      sin(angle),  cos(angle), 0.0,
      0.0, 0.0, 1.0;

  return rotation;
}

Eigen::MatrixXd MainWindow::rotation2rpy( Eigen::MatrixXd rotation )
{
  Eigen::MatrixXd rpy = Eigen::MatrixXd::Zero(3,1);

  rpy.coeffRef(0,0) = atan2( rotation.coeff(2,1), rotation.coeff(2,2) );
  rpy.coeffRef(1,0) = atan2(-rotation.coeff(2,0), sqrt(pow(rotation.coeff(2,1),2) + pow(rotation.coeff(2,2),2)));
  rpy.coeffRef(2,0) = atan2( rotation.coeff(1,0), rotation.coeff(0,0) );

  return rpy;
}

Eigen::MatrixXd MainWindow::rpy2rotation( double roll, double pitch, double yaw )
{
  Eigen::MatrixXd rotation = rotationZ(yaw) * rotationY(pitch) * rotationX(roll);

  return rotation;
}

Eigen::Quaterniond MainWindow::rpy2quaternion( double roll, double pitch, double yaw )
{
  Eigen::MatrixXd rotation = rpy2rotation(roll,pitch,yaw);

  Eigen::Matrix3d rotation3d;
  rotation3d = rotation.block(0,0,3,3);

  Eigen::Quaterniond quaternion;

  quaternion = rotation3d;

  return quaternion;
}

Eigen::Quaterniond MainWindow::rotation2quaternion( Eigen::MatrixXd rotation )
{
  Eigen::Matrix3d rotation3d;

  rotation3d = rotation.block(0,0,3,3);

  Eigen::Quaterniond quaternion;
  quaternion = rotation3d;

  return quaternion;
}

Eigen::MatrixXd MainWindow::quaternion2rpy( Eigen::Quaterniond quaternion )
{
  Eigen::MatrixXd rpy = rotation2rpy(quaternion.toRotationMatrix());

  return rpy;
}

Eigen::MatrixXd MainWindow::quaternion2rotation( Eigen::Quaterniond quaternion )
{
  Eigen::MatrixXd rotation = quaternion.toRotationMatrix();

  return rotation;
}

void MainWindow::parseIniPoseData(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  social_robot_arm_sdk::JointPose msg;

  // parse movement time
  double mov_time = doc["mov_time"].as<double>();
  msg.mov_time = mov_time;

  // parse target pose
  YAML::Node tar_pose_node = doc["tar_pose"];
  for (YAML::iterator it = tar_pose_node.begin(); it != tar_pose_node.end(); ++it)
  {
    std::string joint_name = it->first.as<std::string>();
    double value = it->second.as<double>();

    msg.pose.name.push_back(joint_name);
    msg.pose.position.push_back(value * DEG2RAD);
  }

  qnode.sendJointPoseMsg( msg );
}

void MainWindow::parseSetPoseData(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  social_robot_arm_sdk::JointPose msg;

  // parse movement time
  double mov_time = doc["mov_time"].as<double>();
  msg.mov_time = mov_time;

  // parse target pose
  YAML::Node tar_pose_node = doc["tar_pose"];
  for (YAML::iterator it = tar_pose_node.begin(); it != tar_pose_node.end(); ++it)
  {
    std::string joint_name = it->first.as<std::string>();
    double value = it->second.as<double>();
    if(joint_name == "LShoulder_Pitch") {
      ui.l1_box->setValue( value );
    } else if(joint_name == "LShoulder_Roll") {
      ui.l2_box->setValue( value );
    } else if(joint_name == "LElbow_Pitch") {
      ui.l3_box->setValue( value );
    } else if(joint_name == "LElbow_Yaw") {
      ui.l4_box->setValue( value );
    } else if(joint_name == "LWrist_Pitch") {
      ui.l5_box->setValue( value );
    } else if(joint_name == "LFinger") {
      ui.lf_box->setValue( value );
    } else if(joint_name == "RShoulder_Pitch") {
      ui.r1_box->setValue( value );
    } else if(joint_name == "RShoulder_Roll") {
      ui.r2_box->setValue( value );
    } else if(joint_name == "RElbow_Pitch") {
      ui.r3_box->setValue( value );
    } else if(joint_name == "RElbow_Yaw") {
      ui.r4_box->setValue( value );
    } else if(joint_name == "RWrist_Pitch") {
      ui.r5_box->setValue( value );
    } else if(joint_name == "RFinger") {
      ui.rf_box->setValue( value );
    } else if(joint_name == "Waist_Roll") {
      ui.wr_box->setValue( value );
    } else if(joint_name == "Waist_Pitch") {
      ui.wp_box->setValue( value );
    } else if(joint_name == "Head_Yaw") {
      ui.hy_box->setValue( value );
    } else if(joint_name == "Head_Pitch") {
      ui.hp_box->setValue( value );
    }

  }


}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
  ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
  QMessageBox::about(this, tr("About ..."),tr("<h2>Social Robot Motion GUI 0.20</h2>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::closeEvent(QCloseEvent *event)
{
  QMainWindow::closeEvent(event);
}

void MainWindow::on_button_set_motion_clicked(bool check)
{
  qnode.sendSetMode("arm_motion_control_module");
}

void MainWindow::on_pos_data_save_clicked(bool check)
{
  QString fileqs = ui.pos_data_file->text();
  std::string filePath = ros::package::getPath("social_robot_arm_gui") + "/arm/"+fileqs.toStdString();

  std::ofstream ss(filePath.data());
	if( ss.is_open() ){
    ss << "# time parameter\n\nmov_time : ";
    ss << ui.joint_mov_time_box->value();
    ss << "\t# movement time\n\n";
    ss << "# target pose [deg]\n\n";
    ss << "tar_pose :\n";
    ss << "  LShoulder_Pitch  : " << ui.cr_l1_box->value() << "\n";
    ss << "  LShoulder_Roll   : " << ui.cr_l2_box->value() << "\n";
    ss << "  LElbow_Pitch     : " << ui.cr_l3_box->value() << "\n";
    ss << "  LElbow_Yaw       : " << ui.cr_l4_box->value() << "\n";
    ss << "  LWrist_Pitch     : " << ui.cr_l5_box->value() << "\n";
    ss << "  LFinger          : " << ui.cr_lf_box->value() << "\n";
    ss << "  RShoulder_Pitch  : " << ui.cr_r1_box->value() << "\n";
    ss << "  RShoulder_Roll   : " << ui.cr_r2_box->value() << "\n";
    ss << "  RElbow_Pitch     : " << ui.cr_r3_box->value() << "\n";
    ss << "  RElbow_Yaw       : " << ui.cr_r4_box->value() << "\n";
    ss << "  RWrist_Pitch     : " << ui.cr_r5_box->value() << "\n";
    ss << "  RFinger          : " << ui.cr_rf_box->value() << "\n";
    
    ss.close();

    init_pose_combo();
	}

}

void MainWindow::on_motion_add_file_clicked(bool check)
{
  // std::cout << "on_motion_add_file" << std::endl;
  QListWidgetItem *newItem = new QListWidgetItem;

  std::string file_name = ui.motion_add_file_combo->currentText().toStdString();
  double time_name = ui.motion_time_edit->value();
  std::stringstream ss;
  ss << file_name << "," << time_name;
  std::string item_text = ss.str();
  newItem->setText(QString::fromStdString(item_text));
  ui.motion_list->insertItem(ui.motion_list->count(), newItem);
}

void MainWindow::on_motion_remove_file_clicked(bool check)
{
  // std::cout << "on_motion_remove_file_clicked" << std::endl;
  // ui.motion_list->removeItemWidget(ui.motion_list->selectedItems().at(0));
  QList<QListWidgetItem*> items = ui.motion_list->selectedItems();

  // foreach(QListWidgetItem* item, items){
  for (int ix = 0; ix < items.count(); ix++) {
    QListWidgetItem* item = items.at(ix);
    ui.motion_list->removeItemWidget(item);
    delete item; // Qt documentation warnings you to destroy item to effectively remove it from QListWidget.
  }

}

void MainWindow::on_motion_file_play_test_clicked(bool check)
{
  std::vector<std::string> file_times;
  for(int ix = 0; ix < ui.motion_list->count(); ix++) {
    QListWidgetItem* item = ui.motion_list->item(ix);
    file_times.push_back(item->text().toStdString());
  }

  std::vector<std::string> hw_file_times;
  for(int ix = 0; ix < ui.hw_motion_list->count(); ix++) {
    QListWidgetItem* item = ui.hw_motion_list->item(ix);
    hw_file_times.push_back(item->text().toStdString());
  }
  qnode.sendPlayMotionMsg(file_times, hw_file_times, "");
}


void MainWindow::on_motion_file_play_save_clicked(bool check) {
  std::string save_name = ui.motion_file_play_save_edit->text().toStdString();
  std::vector<std::string> file_times;
  for(int ix = 0; ix < ui.motion_list->count(); ix++) {
    QListWidgetItem* item = ui.motion_list->item(ix);
    file_times.push_back(item->text().toStdString());
  }
  std::vector<std::string> hw_file_times;
  for(int ix = 0; ix < ui.hw_motion_list->count(); ix++) {
    QListWidgetItem* item = ui.hw_motion_list->item(ix);
    hw_file_times.push_back(item->text().toStdString());
  }

  qnode.sendPlayMotionMsg(file_times, hw_file_times, save_name);
}

void MainWindow::on_hw_pos_data_save_clicked(bool check)
{
  QString fileqs = ui.hw_pos_data_file->text();
  std::string filePath = ros::package::getPath("social_robot_arm_gui") + "/head_waist/"+fileqs.toStdString();

  std::ofstream ss(filePath.data());
	if( ss.is_open() ){
    ss << "# time parameter\n\nmov_time : ";
    ss << ui.joint_mov_time_box->value();
    ss << "\t# movement time\n\n";
    ss << "# target pose [deg]\n\n";
    ss << "tar_pose :\n";
    ss << "  Waist_Roll   : " << ui.cr_wr_box->value() << "\n";
    ss << "  Waist_Pitch  : " << ui.cr_wp_box->value() << "\n";
    ss << "  Head_Yaw     : " << ui.cr_hy_box->value() << "\n";
    ss << "  Head_Pitch   : " << ui.cr_hp_box->value() << "\n";
    ss.close();

    init_pose_combo();
	}

}

void MainWindow::on_hw_motion_add_file_clicked(bool check)
{
  // std::cout << "on_motion_add_file" << std::endl;
  QListWidgetItem *newItem = new QListWidgetItem;

  std::string file_name = ui.hw_motion_add_file_combo->currentText().toStdString();
  double time_name = ui.hw_motion_time_edit->value();
  std::stringstream ss;
  ss << file_name << "," << time_name;
  std::string item_text = ss.str();
  newItem->setText(QString::fromStdString(item_text));
  ui.hw_motion_list->insertItem(ui.hw_motion_list->count(), newItem);
}

void MainWindow::on_hw_motion_remove_file_clicked(bool check)
{
  // std::cout << "on_motion_remove_file_clicked" << std::endl;
  // ui.motion_list->removeItemWidget(ui.motion_list->selectedItems().at(0));
  QList<QListWidgetItem*> items = ui.hw_motion_list->selectedItems();

  // foreach(QListWidgetItem* item, items){
  for (int ix = 0; ix < items.count(); ix++) {
    QListWidgetItem* item = items.at(ix);
    ui.hw_motion_list->removeItemWidget(item);
    delete item; // Qt documentation warnings you to destroy item to effectively remove it from QListWidget.
  }

}


}  // namespace social_robot_arm_gui

