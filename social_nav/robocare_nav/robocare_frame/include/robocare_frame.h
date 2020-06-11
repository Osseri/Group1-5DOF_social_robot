#ifndef ROBOCARE_FRAME_H_
#define ROBOCARE_FRAME_H_

#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class robocare_frame
{
public:
	robocare_frame();
	~robocare_frame();
	
private:
	ros::NodeHandle m_nodeHandle;
	ros::Subscriber m_sub_odom;
	void odom_SubscribeCallBack(const nav_msgs::Odometry::ConstPtr& input);

};

#endif
