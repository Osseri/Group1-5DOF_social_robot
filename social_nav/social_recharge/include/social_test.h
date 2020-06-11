#ifndef SOCIAL_TEST_H
#define SOCIAL_TEST_H

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
#include <std_srvs/Empty.h>
#include <std_msgs/Int8MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

//bkk
#include "tool_class.h"
#include <fcntl.h>          // FILE control
#include <termios.h>        // Terminal IO
#include <signal.h>
#include <cstdlib>
#include <unistd.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

/*****************************************************************************
 ** Defines
 *****************************************************************************/
#define ESC_ASCII_VALUE             0x1b
#define EXIT_LOOP                   0x71
#define GO_50_CM                    0x77    // W , w
#define GO_100_CM                   0x65    // E , e
#define GO_300_CM                   0x72    // R , r
#define STOP                        0x73    // S , s
#define BACK_HOME                   0x64    // D , d
#define MAPMODE                     0x78    // X , x
#define MAPSAVE                     0x63    // C , c
#define STATION_POSE_SET            0x31    // 1 , !
#define GAME_POSE_SET               0x32    // 2 , @
#define DOOR_POSE_SET               0x33    // 3 , #


using namespace std;

class social_test
{

private:
    eun_u::tool_class mat_cal;
    ros::NodeHandle m_nodeHandle;

    // Publish
    ros::ServiceClient m_ser_clearCostmap;  // Costmap Clear

    ros::Publisher m_ptr_pub;
    ros::Publisher cmd_vel_pub;     //
    ros::Publisher m_pub_path;      // path 경로 publish
    ros::Publisher m_pub_init;
    ros::Publisher m_pub_wheel;
    ros::Publisher m_pub_goal;
    // Subscribe

    ros::Subscriber sub_map_mode;
    ros::Subscriber m_sub_get_pose;
    ros::Subscriber m_sub_pose;
    ros::Subscriber m_sub_odom;
    ros::Subscriber m_sub_init_button;
    ros::Subscriber m_sub_goal_button;

    // ROS msgs
    geometry_msgs::PoseStamped _goal_ptr;
    geometry_msgs::Twist _twist_msg;
    geometry_msgs::Pose _pose_msg;


public:
    social_test();
    virtual ~social_test();

    //init_node
    void init_node();
    void key_input();

    //CallBack

    static void* thread_key_input(void* arg);
    static void* thread_stopping_test(void* arg);
    void pose_SubscribeCallBack(const nav_msgs::Odometry::ConstPtr& input);	//Odom값 입력
    void get_pose_SubscribeCallBack(const geometry_msgs::PoseWithCovarianceStamped input);
    void go_to_init_SubscribeCallBack(const std_msgs::Bool& input);
    void go_to_goal_SubscribeCallBack(const std_msgs::Bool& input);

    int getch(void);
    int kbhit(void);

    move_base_msgs::MoveBaseActionGoal m_goal;
    void go_to_goal(double x, double y, double angle, double w);
    void wheel_forward_test(double distance);
    void wheel_backward_test(double distance);
    void rotation(double angle);		//로봇의 회전
    void wheelmoveX(double distance, double speed);		//로봇의 이동.
    void wheel_stop();

    eun_u::Point4D m_get_pose;              // Odom값 저장 x, y위치와 z: 로봇 방향 값
    eun_u::Point3D m_pose;

};

#endif // SOCIAL_TEST_H
