/*
 * robocare_navigation.h
 *
 *  Created on: 2019. 5. 2.
 *      Author: kkbaek
 */


#ifndef __ROBOCARE_NAVIGATION_H__
#define __ROBOCARE_NAVIGATION_H__

#include <ros/ros.h>
#include <sstream>
#include "tool_class.h"
#include <tf/transform_broadcaster.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

//Navigation Msg & Srv
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <robocare_msgs/NavigationStatus.h>
#include <robocare_msgs/ChangeMap.h>

//ros Srv &Msg
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>

#define	MOVING 0
#define	FINISHED 1
#define	CANCELED 2
#define	PAUSED 3
#define	UNKNOWN_ERROR 10
#define	PLANNING_FAILED 11
#define	OCCUFIED 12

using namespace std;

namespace robocare
{

	class robocare_navigation
	{

	private:
		eun_u::tool_class mat_cal;
		ros::NodeHandle m_nodeHandle;

		//Navigation(Sub)
		ros::Subscriber m_sub_goal;
		ros::Subscriber m_sub_stop;
		ros::Subscriber m_sub_init;

		//Navigation(Pub)
		ros::Publisher m_pub_costmap;
		ros::Publisher m_pub_status;

		//Navigation(Req)
		ros::ServiceServer m_req_ChangMap;

		//Ros Navigation(sub)
		ros::Subscriber m_sub_pose;
		ros::Subscriber m_sub_costmap;
		ros::Subscriber m_sub_result;
		ros::Subscriber m_sub_status;

		//Ros Navigation(pub)
		ros::Publisher m_pub_init;
		ros::Publisher m_pub_goal;
		ros::Publisher m_pub_stop;
		ros::Publisher m_pub_amcl_map;
		ros::Publisher m_pub_cost_map;

		//Ros Navigation(req)
		ros::ServiceClient m_ser_clearCostMap;

		//Sensor & Wheel
		ros::Subscriber m_sub_odom;
		ros::Subscriber m_sub_laser;
		ros::Publisher m_pub_laser;
		ros::Subscriber m_sub_wheel;
		ros::Publisher m_pub_wheel;

	public:
		robocare_navigation();
		virtual ~robocare_navigation();

		//Ros node init
		void init_node();

		//Navigation(Sub)
		void set_pose(geometry_msgs::PoseWithCovarianceStamped input);

		move_base_msgs::MoveBaseActionGoal m_goal;
		void go_to_goal(geometry_msgs::PoseWithCovarianceStamped input_msg);

		void stop(std_msgs::Empty input);

		//Navigation(Pub)
		void pub_cost_map(nav_msgs::OccupancyGrid input);

		//Navigation(Req)
		bool change_map_ServiceCallback(robocare_msgs::ChangeMap::Request &request, robocare_msgs::ChangeMap::Response &response);

		//Ros Navigation(sub)
		eun_u::Point3D m_cur_pose;
		void get_pose(geometry_msgs::PoseWithCovarianceStamped input);

		void get_goal(move_base_msgs::MoveBaseActionGoal input);

		int m_width;
		int m_height;
		vector<int> m_costmap; //0 ~ 100 0이면 주행 가능.
		void get_costmap(nav_msgs::OccupancyGrid input);

		bool m_movig_flag;
		void get_status(actionlib_msgs::GoalStatusArray input);	 //이동 중  Status 전달을 위히 필요

		int m_status;
		void get_result(move_base_msgs::MoveBaseActionResult input);

		//Escape Local Minimum
		bool m_escape_dead_zone;
		double m_robot_radius;
		bool robot_in_wall();
		void escape_dead_zone();
		void range_safety_direction(bool* output); //[0] 전 [1] 좌 [2] 우
		void costmap_safety_direction(bool* output); //[0] 전 [1] 좌 [2] 우
		void rotation(double angle);
		void wheelmoveX(double distance);
		bool check_forward_obstacle();

		//Sensor & wheel
		eun_u::Point3D m_odom_pose;
		void odom_SubscribeCallBack(const nav_msgs::Odometry::ConstPtr& input);

		double m_range_max;
		sensor_msgs::LaserScan m_laser;
		geometry_msgs::TransformStamped m_laser_to_base;
		void laser_data_SubscribeCallBack(const sensor_msgs::LaserScan input);

		void cmd_vel_SubscribeCallBack(const geometry_msgs::Twist::ConstPtr& input);
	};
}

#endif /* __SILBOT3_NAVIGATION_H__ */

