#include "Config.h"
#include <fstream>
#include <pthread.h>

//Ros
#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>

//Navigation Msg
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <robocare_msgs/NavigationStatus.h>

//ros Srv &Msg
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <nav_msgs/GetPlan.h>

using namespace std;

class hccb_navigation_test
{
public:
	hccb_navigation_test();
	~hccb_navigation_test();

private:
	//Ros
	ros::NodeHandle m_nodeHandle;
	void initNode();

	//Ros Srv & Msg
	bool m_go_and_stop;
	double m_get_status_time;
	geometry_msgs::PoseWithCovarianceStamped m_robot_pose;
	ros::ServiceClient m_make_plan;
	ros::Subscriber m_sub_status;
	ros::Subscriber m_sub_pose;
	ros::Publisher m_pub_goal;
	ros::Publisher m_pub_stop;

	void status_SubscribeCallBack(hccb_msgs::NavigationStatus input);
	void pose_SubscribeCallBack(geometry_msgs::PoseWithCovarianceStamped input);

	//Function
	void gotogoal(int input);
	void stop();

	//Test
	static void* thread_test(void* arg);
	void test();
};
