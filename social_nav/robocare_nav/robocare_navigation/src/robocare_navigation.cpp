/*
 * Hccb_navigation.cpp
 *
 *  Created on: 2019. 5. 2.
 *      Author: BKK
 */

#include "robocare_navigation.h"
#include <math.h>

using namespace robocare;

robocare_navigation::robocare_navigation()
{
	//Ros Navigation(sub)
	m_cur_pose.init();
	m_width = 0;
	m_height = 0;
	if((int) m_costmap.size() > 0) m_costmap.clear();
	m_movig_flag = false;
	m_status = FINISHED;

	//Escape Local Minimum
	string laser_path;
	m_nodeHandle.param<std::string>("robocare_navigation/laser_frame", laser_path, "laser");
	m_nodeHandle.param("robocare_navigation/escape_dead_zone", m_escape_dead_zone, true);
	m_nodeHandle.param("robocare_navigation/robot_radius", m_robot_radius, 0.25);

	tf2_ros::Buffer tf2_Buffer;
	tf2_ros::TransformListener tf2_listener(tf2_Buffer);
	m_laser_to_base = tf2_Buffer.lookupTransform("base_link", laser_path.c_str(),	ros::Time(0), ros::Duration(1.0));

	//Sensor & wheel
	m_odom_pose.init();
	m_nodeHandle.param("robocare_navigation/range_max", m_range_max, 5.0);

   init_node();
}

robocare_navigation::~robocare_navigation()
{
	if((int) m_costmap.size() > 0) m_costmap.clear();
}

//Ros node init
void robocare_navigation::init_node()
{
	//Navigation(Sub)
	m_sub_init = m_nodeHandle.subscribe("/robocare/navigation/set_pose", 1000, &robocare_navigation::set_pose, this); //위치 설정
	m_sub_goal = m_nodeHandle.subscribe("/robocare/navigation/go_to_goal", 1000, &robocare_navigation::go_to_goal, this); //주행시작
	m_sub_stop = m_nodeHandle.subscribe("/robocare/navigation/stop", 1000, &robocare_navigation::stop, this); //주행정지

	//Navigation(Pub)
	m_pub_costmap  = m_nodeHandle.advertise<nav_msgs::OccupancyGrid>("robocare/navigation/costmap_image", 10, true);
	m_pub_status = m_nodeHandle.advertise<robocare_msgs::NavigationStatus>("/robocare/navigation/status", 1000);

	//Navigation(Req)
	m_req_ChangMap = m_nodeHandle.advertiseService("/robocare/navigation/change_map", &robocare_navigation::change_map_ServiceCallback, this);

	//Ros Navigation(sub)
	m_sub_pose = m_nodeHandle.subscribe("/amcl_pose", 1000, &robocare_navigation::get_pose, this);
	m_sub_goal = m_nodeHandle.subscribe("/move_base/goal", 1000, &robocare_navigation::get_goal, this);
	m_sub_costmap = m_nodeHandle.subscribe("/move_base/global_costmap/costmap", 1, &robocare_navigation::get_costmap, this);
	m_sub_status = m_nodeHandle.subscribe("/move_base/status", 1, &robocare_navigation::get_status, this);
	m_sub_result = m_nodeHandle.subscribe("/move_base/result", 1, &robocare_navigation::get_result, this); //주행 결과

	//Ros Navigation(pub)
	m_pub_init = m_nodeHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1000);
	m_pub_goal = m_nodeHandle.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1000);
	m_pub_stop = m_nodeHandle.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1000);
	m_pub_amcl_map = m_nodeHandle.advertise<std_msgs::String>("/map_amcl_reload", 1000);
	m_pub_cost_map = m_nodeHandle.advertise<std_msgs::String>("/map_cost_reload", 1000);

	//Ros Navigation(req)
	m_ser_clearCostMap = m_nodeHandle.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

	//Sensor & Wheel
	m_sub_odom = m_nodeHandle.subscribe("/odom", 1, &robocare_navigation::odom_SubscribeCallBack, this);
    m_sub_laser = m_nodeHandle.subscribe<sensor_msgs::LaserScan>("/urg_node/scan", 1, &robocare_navigation::laser_data_SubscribeCallBack, this);
	m_pub_laser = m_nodeHandle.advertise<sensor_msgs::LaserScan>("/robocare/scan", 1000);
	m_sub_wheel = m_nodeHandle.subscribe<geometry_msgs::Twist>("/ros/cmd_vel", 1, &robocare_navigation::cmd_vel_SubscribeCallBack, this);
	m_pub_wheel = this->m_nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

//Navigation(Sub)
void robocare_navigation::set_pose(geometry_msgs::PoseWithCovarianceStamped input)
{
	m_pub_init.publish(input);
}

void robocare_navigation::go_to_goal(geometry_msgs::PoseWithCovarianceStamped input_msg)
{
	printf("Set Goal!\n");
	std_srvs::Empty em;
	m_ser_clearCostMap.call(em);
	ros::Duration(0.5).sleep();

	m_goal.goal.target_pose.header.frame_id = "map";
	m_goal.goal.target_pose.header.stamp = ros::Time::now();

	m_goal.goal.target_pose.pose.position.x = input_msg.pose.pose.position.x;
	m_goal.goal.target_pose.pose.position.y = input_msg.pose.pose.position.y;
	m_goal.goal.target_pose.pose.position.z = input_msg.pose.pose.position.z;
	m_goal.goal.target_pose.pose.orientation.x = input_msg.pose.pose.orientation.x;
	m_goal.goal.target_pose.pose.orientation.y = input_msg.pose.pose.orientation.y;
	m_goal.goal.target_pose.pose.orientation.z = input_msg.pose.pose.orientation.z;
	m_goal.goal.target_pose.pose.orientation.w = input_msg.pose.pose.orientation.w;

	m_pub_goal.publish(m_goal);
}

void robocare_navigation::stop(std_msgs::Empty input)
{
	ROS_INFO("Navigation Stop");
	actionlib_msgs::GoalID goal;
	m_pub_stop.publish(goal);
}

//Navigation(Pub)
void robocare_navigation::pub_cost_map(nav_msgs::OccupancyGrid input)
{
	nav_msgs::OccupancyGrid msg;
	ros::Time time = ros::Time::now();
	msg.header.stamp = time;
	msg.header.frame_id = "map";
	msg.info = input.info;

	for(int i=0; i < (int) m_costmap.size(); i++) msg.data.push_back(m_costmap[i]);
	m_pub_costmap.publish(msg);
}

//Navigation(Req)
bool robocare_navigation::change_map_ServiceCallback(robocare_msgs::ChangeMap::Request &request, robocare_msgs::ChangeMap::Response &response)
{
	string input_path = request.data;
	string folder_name = request.data;
	std::string delimiter = "/";
	while(1)
	{
		size_t pos = 0;
		pos = folder_name.find(delimiter);
		if(pos == std::string::npos) break;
		folder_name.erase(0, pos + delimiter.length());
	}

	string amcl_path = input_path + "/" + "amcl.yaml";
	string cost_path = input_path + "/" + "cost.yaml";

	std_msgs::String msg;

	msg.data = amcl_path;
	m_pub_amcl_map.publish(msg);

	msg.data = cost_path;
	m_pub_cost_map.publish(msg);

	cout << "amcl_path : " << amcl_path << endl;
	cout << "cost_path : " << cost_path << endl;

	response.success = true;
	ros::Duration(10.0).sleep();
	return true;
}

//Ros Navigation(sub)
void robocare_navigation::get_pose(geometry_msgs::PoseWithCovarianceStamped input)
{
	m_cur_pose.x = input.pose.pose.position.x;
	m_cur_pose.y = input.pose.pose.position.y;
	m_cur_pose.z = input.pose.pose.position.z;
}

void robocare_navigation::get_goal(move_base_msgs::MoveBaseActionGoal input)
{
	m_goal = input;
}

void robocare_navigation::get_costmap(nav_msgs::OccupancyGrid input)
{
	if((int) m_costmap.size() > 0) m_costmap.clear();
	int map_size = input.info.width * input.info.height;
	m_width = input.info.width;
	m_height = input.info.height;
	for(int i = 0; i < map_size; i++) m_costmap.push_back((int) input.data[i]);
	pub_cost_map(input);
}

void robocare_navigation::get_status(actionlib_msgs::GoalStatusArray input)
{
	if((int) input.status_list.size() > 0)
	{
		int index = input.status_list.size() - 1;
		if(input.status_list[index].status == 1 && m_movig_flag == false)
		{
			ROS_INFO("Robot Moving");
			m_movig_flag = true;
			ros::Duration(0.2).sleep();
			robocare_msgs::NavigationStatus start_msg;
			start_msg.status = start_msg.STATUS_MOVING;
			m_pub_status.publish(start_msg);
		}
	}
}

void robocare_navigation::get_result(move_base_msgs::MoveBaseActionResult input)
{
	string	 status;
	status = input.status.text;
	std::string finish = "reached";
	std::string planning_failed = "Failed";
	std::string oscillating = "oscillating";

	size_t pos = 0;
	robocare_msgs::NavigationStatus msg;

	if(status.find(finish) != std::string::npos)
	{
		ROS_INFO("Navigation Finished");
		msg.status = msg.STATUS_FINISHED;
		m_pub_status.publish(msg);
		m_movig_flag = false;
	}

	else if(status.find(planning_failed) != std::string::npos)
	{
		msg.status = msg.STATUS_PAUSED;
		m_pub_status.publish(msg);
		ros::Duration(0.05).sleep();

		if(robot_in_wall() == true && m_escape_dead_zone == true)
		{
			escape_dead_zone();
			ROS_INFO("Retry");
		}

		else
		{
			ROS_INFO("Planning Failed & Retry");
		}

		std_srvs::Empty em;
		m_ser_clearCostMap.call(em);
		ros::Duration(0.1).sleep();
		m_goal.goal.target_pose.header.stamp = ros::Time::now();
		m_pub_goal.publish(m_goal);
	}

	else if(status.find(oscillating) != std::string::npos)
	{
		ROS_INFO("Oscillating and Retry");
		ros::Duration(0.2).sleep();
		m_goal.goal.target_pose.header.stamp = ros::Time::now();
		m_pub_goal.publish(m_goal);
	}

	else if(input.status.status == 2)
	{
		ROS_INFO("Navigation Canceled");
		msg.status = msg.STATUS_CANCELED;
		m_pub_status.publish(msg);
		m_movig_flag = false;
	}

	else
	{
		ROS_INFO("UNKNOWN_ERROR");
		cout << status.c_str() << endl;
	}
}

//Escape Local Minimum
bool robocare_navigation::robot_in_wall()
{
	int index;
	int w_index, h_index;
	int robot_width = (int) (m_cur_pose.x / 0.05);
	int robot_height = (int) (m_cur_pose.y / 0.05);

	ROS_INFO("Robot W : %d\tH : %d", robot_width, robot_height);
	if(robot_width > 0 && robot_width < m_width && robot_height > 0 && robot_height < m_height)
	{
		for(int i = -10; i < 11; i++)
		{
			for(int j = -10; j < 11; j++)
			{
				w_index = robot_width + i;
				h_index = robot_height + j;
				if(w_index > 0 && w_index < m_width && h_index > 0 && h_index < m_height)
				{
					index = h_index * m_width + w_index;
					if(m_costmap[index] != 0)
					{
						ROS_INFO("Planning Failed and Robot near the wall");
						return true;
					}
					else
					{
						return false;
					}
				}
			}
		}
	}

	else return false;
}

void robocare_navigation::escape_dead_zone()
{
	bool dir[3];
	int count[3];
	double cur_time;
	memset(count, 0, sizeof(int) * 3);
	ROS_INFO("Escape Dead Zone Start");

	cur_time = ros::Time::now().toSec();
	ros::Rate r(9);
	while(1)
	{
		range_safety_direction(dir);
		if(dir[0] == true)	count[0]++;
		else count[0] = 0;
		if(dir[1] == true)	count[1]++;
		else count[1] = 0;
		if(dir[2] == true)	count[2]++;
		else count[3] = 0;

		if(count[0] > 10) break;
		if(count[1] > 10) break;
		if(count[2] > 10) break;
		if(ros::Time::now().toSec() - cur_time > 5.0)
		{
			ROS_INFO("No Escape Way");
			break;
		}
		r.sleep();
	}

	if(count[0] > 10 || count[1] > 10 || count[2] > 10)
	{
		if(count[0] > 10)
		{
			ROS_INFO("Go Forward");
		}

		else if(count[1] > 10)
		{
			ROS_INFO("Go Left");
			rotation(90);

		}

		else if(count[2] > 10)
		{
			ROS_INFO("Go Right");
			rotation(-90);
		}

		wheelmoveX(0.5);
	}

	else
	{
		ROS_INFO("Go Back");
		rotation(180);
		wheelmoveX(0.5);
	}
}

void robocare_navigation::range_safety_direction(bool* output)
{
	double min_range = -(m_robot_radius + 0.2);
	double max_range = m_robot_radius + 0.2;
	double move_dis = max_range + 0.3;

	vector<eun_u::Point2D> laser_points;
	vector<eun_u::Point2D> base_points;
	mat_cal.convertRangeToPoints(m_laser, laser_points);
	mat_cal.TranslationPointsByTF(m_laser_to_base, laser_points, base_points);

	for(int i = 0; i < 3; i++) output[i] = true;
	for(int i = 0; i < (int) base_points.size(); i++)
	{
		if(base_points[i].y > min_range && base_points[i].y < max_range && base_points[i].x > 0 && base_points[i].x < move_dis) output[0] = false;
		else if(base_points[i].x > min_range && base_points[i].x < max_range && base_points[i].y > 0 && base_points[i].y < move_dis) output[1] = false;
		else if(base_points[i].x > min_range && base_points[i].x < max_range && base_points[i].y < 0 && base_points[i].y > -move_dis) output[2] = false;
	}

	if((int) laser_points.size() > 0) laser_points.clear();
	if((int) base_points.size() > 0) base_points.clear();
}

void robocare_navigation::rotation(double angle)
{
	double target_angle = m_odom_pose.z + angle;
	if(target_angle > 180) target_angle = target_angle - 360;
	else if(target_angle < -180) target_angle = target_angle + 360;

	//printf("Target Angle : %0.3lf\tStart Angle : %0.3lf\n", target_angle, m_odom_pose.z);

	double bet_angle;
	ros::Rate r(9);
	while(ros::ok())
	{
		geometry_msgs::Twist msg;
		msg.linear.x = msg.linear.y = msg.linear.z = 0;
		msg.angular.x = msg.angular.y = msg.angular.z = 0;

		bet_angle = target_angle - m_odom_pose.z;
		if(bet_angle > 180) bet_angle = bet_angle - 360;
		else if(bet_angle < -180) bet_angle = bet_angle + 360;

		//printf("Target Angle : %0.3lf\tCur Angle : %0.3lf\n", target_angle, m_odom_pose.z);
		if(fabs(bet_angle) > 30)
		{
			if(angle > 0) msg.angular.z = 1.0;
			else msg.angular.z = -1.0;
			m_pub_wheel.publish(msg);
		}

		else if(fabs(bet_angle) > 5)
		{
			if(angle > 0) msg.angular.z = 0.2;
			else msg.angular.z = -0.2;
			m_pub_wheel.publish(msg);
		}

		else
		{
			m_pub_wheel.publish(msg);
			break;
		}
		r.sleep();
	}
}

void robocare_navigation::wheelmoveX(double distance)
{
	ROS_INFO("MoveX Start");
	eun_u::Point2D robot_pose;
	eun_u::Point2D start_pose;
	double move_dis;
	double start_time = ros::Time::now().toSec();

	start_pose.x = m_odom_pose.x;
	start_pose.y = m_odom_pose.y;

	ros::Rate r(9);
	while(ros::ok())
	{
		geometry_msgs::Twist msg;
		robot_pose.x = m_odom_pose.x;
		robot_pose.y = m_odom_pose.y;

		move_dis = mat_cal.distance_BetweenPoint2DAndPoint2D(robot_pose, start_pose);

		if(move_dis > fabs(distance) || ros::Time::now().toSec() - start_time > 10)
		{
			if(move_dis > fabs(distance)) ROS_INFO("MoveX Get reached");
			else ROS_INFO("MoveX Time Over(10 second)");

			msg.linear.x = msg.linear.y = msg.linear.z = 0;
			msg.angular.x = msg.angular.y = msg.angular.z = 0;
			m_pub_wheel.publish(msg);
			break;
		}

		else if(check_forward_obstacle() == true)
		{
			ROS_INFO("MoveX Obstacle");
			msg.linear.x = msg.linear.y = msg.linear.z = 0;
			msg.angular.x = msg.angular.y = msg.angular.z = 0;
			m_pub_wheel.publish(msg);
		}

		else
		{
			ROS_INFO("Move_Dis : %0.3lfTarget_Dis : %0.3lf", move_dis, distance);
			msg.linear.x = 0.15;
			m_pub_wheel.publish(msg);
		}

		r.sleep();
	}
}

bool robocare_navigation::check_forward_obstacle()
{
	bool flag = false;
	double robot_range = m_robot_radius + 0.2;
	double move_dis = m_robot_radius + 0.5;

	vector<eun_u::Point2D> laser_points;
	vector<eun_u::Point2D> base_points;
	mat_cal.convertRangeToPoints(m_laser, laser_points);
	mat_cal.TranslationPointsByTF(m_laser_to_base, laser_points, base_points);

	for(int i = 0; i < (int) base_points.size(); i++)
	{
		if(base_points[i].y > -robot_range && base_points[i].y < robot_range && base_points[i].x > 0 && base_points[i].x < move_dis) flag = true;
	}

	if((int) laser_points.size() > 0) laser_points.clear();
	if((int) base_points.size() > 0) base_points.clear();
	return flag;
}

//Sensor
void robocare_navigation::odom_SubscribeCallBack(const nav_msgs::Odometry::ConstPtr& input)
{
	eun_u::Point3D rotation;
	rotation = mat_cal.QuaternionToEuler(input->pose.pose.orientation.x, input->pose.pose.orientation.y, input->pose.pose.orientation.z, input->pose.pose.orientation.w);
	m_odom_pose.x = input->pose.pose.position.x;
	m_odom_pose.y = input->pose.pose.position.y;
	m_odom_pose.z = rotation.z;
	ros::Duration(0.01).sleep();
}

void robocare_navigation::laser_data_SubscribeCallBack(const sensor_msgs::LaserScan input)
{
	sensor_msgs::LaserScan out_put;
	m_laser = out_put = input;

	if((int) input.ranges.size() > 0)
	{
		int count = 0;
		double angle;
		double range;
		eun_u::Point2D axis;
		for(int i = 1; i < (int) out_put.ranges.size(); i++)
		{
			if(input.ranges[i] < input.range_min || input.ranges[i] > m_range_max) out_put.ranges[i] = m_range_max;
		}
	}

	m_pub_laser.publish(out_put);
	ros::Duration(0.01).sleep();
}

void robocare_navigation::cmd_vel_SubscribeCallBack(const geometry_msgs::Twist::ConstPtr& input)
{
	geometry_msgs::Twist msg;

	msg.linear = input->linear;
	msg.linear.y = 0.0;
	msg.angular = input->angular;
	m_pub_wheel.publish(msg);
}

int main(int argc, char** argv)
{
	ROS_INFO("Start Robocare Navigation");
	ros::init(argc, argv, "robocare_navigation");

	robocare_navigation GGG;

	ros::MultiThreadedSpinner spinner(7); // Use 4 threads
	spinner.spin(); // spin() will not return until the node has been shutdown

	return 0;
}
