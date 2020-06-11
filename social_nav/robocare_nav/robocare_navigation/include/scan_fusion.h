/*
 * scan_fusion.h
 *
 *  Created on: 2019. 4. 1.
 *      Author: kkbaek
 */

#ifndef INCLUDE_SCAN_FUSION_H_
#define INCLUDE_SCAN_FUSION_H_

//Ros
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/LaserScan.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tool_class.h"

class scan_fusion
{
public:
	scan_fusion();
	~scan_fusion();

private:
	//tool_class
	eun_u::tool_class mat_cal;

	//Ros
	ros::NodeHandle m_nodeHandle;

	//Ros Srv & Msg
	ros::Subscriber m_sub_laser;
	ros::Subscriber m_sub_astra;
	ros::Publisher m_pub_data;
	vector<eun_u::Point2D> m_astra;

	void laser_SubscribeCallBack(sensor_msgs::LaserScan input);
	void astra_SubscribeCallBack(sensor_msgs::LaserScan input);

	//Fusion
	void fusion(sensor_msgs::LaserScan laser);
	void pub_fusion_data();
};



#endif /* INCLUDE_SCAN_FUSION_H_ */
