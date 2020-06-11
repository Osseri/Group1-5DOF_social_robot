/*
 * silbot3_navigation.h
 *
 *  Created on: 2018. 8. 20.
 *      Author: kk_baek
 */


#include <ros/ros.h>
#include <sstream>
#include <tf/transform_broadcaster.h>

//ros Srv & Msg
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

//robocare_msgs
#include <robocare_msgs/DockOnStation.h>
#include <robocare_msgs/LeaveStation.h>

#define COMMAND_WHEEL_STOP                              "WHEEL_STOP"
#define COMMAND_WHEEL_MOVE_BY_VELOCITY_XYT		"WHEEL_MOVE_BY_VELOCITY_XYT"

//bkk
#include "tool_class.h"

using namespace std;

class social_recharge
{

private:
  eun_u::tool_class mat_cal;
  ros::NodeHandle m_nodeHandle;

  //Navigation
  ros::ServiceServer m_req_docking;
  ros::ServiceServer m_req_leaving;
  ros::Subscriber m_sub_ir_right;
  ros::Subscriber m_sub_ir_left;
  ros::Subscriber m_sub_charge;
  ros::Subscriber m_sub_laser_scan;
  ros::Subscriber m_sub_odom;
  ros::Subscriber m_sub_infrared_image;

  ros::Publisher m_pub_circle_image;
  ros::Publisher m_pub_marker;
  ros::Publisher m_pub_wheel;
  ros::Publisher m_pub_dock;

public:
  social_recharge();
  virtual ~social_recharge();

  //init_node
  void init_node();

  //CallBack
  void charge_SubscribeCallBack(const std_msgs::Bool& input);	//충전 Flag 입력
  void laser_SubscribeCallBack(const sensor_msgs::LaserScan::ConstPtr& input);		//Laser 센서값 입력
  void odom_SubscribeCallBack(const nav_msgs::Odometry::ConstPtr& input);	//Odom값 입력
  bool docking_ServiceCallback(robocare_msgs::DockOnStation::Request &request, robocare_msgs::DockOnStation::Response &response);	//자동충전 Docking 실행
  bool leaving_ServiceCallback(robocare_msgs::LeaveStation::Request &request, robocare_msgs::LeaveStation::Response &response);		//자동충전 Leave 실행

  //Sensor_data
  bool m_charge;	//충전 Flag
  eun_u::Point3D m_pose;		//Odom값 저장 x, y위치와 z: 로봇 방향 값
  eun_u::Bounds3D m_laser_area;	//유효한 Laser 영역 설정
  vector<eun_u::Point2D> m_laser;	//Laser Points값 저장
  std_msgs::Bool docking_msg;

  //Display
  void line_display(eun_u::Line2D tarket, vector<eun_u::Line2D> line);	//인식되 Line Display

  //parking
  double m_tarket_dis;	//인식할 충전기의 길이.
  double m_leave_dis;		//도킹해제할 때 이동 거리.
  double m_trans_threshold;		//Laser인식 이동 Goal 문턱치.
  double m_angle_threshold;		//Laser인식 각도 Goal 문턱치.
  void laser_moving();	//Laser로 인식된 충전기의 위치를 토대로 이동.
  void dock_publish();

  eun_u::Point3D find_target(vector<eun_u::Point2D> laser);		//입력된 Laser 포인트에서 충전기 위치를 추출
  void detect_line(vector<eun_u::Point2D> points, vector<eun_u::Line2D>& out_put);	//입력된 Laser값을 토대로 Line 추출
  eun_u::Line2D select_line(vector<eun_u::Line2D>& input);	//추출된 라인에서 충전기 인식.
  eun_u::Point3D cal_tarket_position(eun_u::Line2D input);	//충전기 Line에서 충전기 위치 추출.
  void wheel_command_laser_rough(eun_u::Point3D position);	//충전기 위치를 이용 도킹위치로 이동(러프이동)
  void wheel_command_laser_detail(eun_u::Point3D position); //충전기 위치를 이용 도킹위치로 이동(정밀이동)
  void rotation(double angle);		//로봇의 회전
  void wheelmoveX(double distance, double speed);		//로봇의 이동.
};


