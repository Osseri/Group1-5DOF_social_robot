
#include <robocare_navigation_test.h>
#include <ros/package.h>

hccb_navigation_test::hccb_navigation_test()
{
	m_go_and_stop = false;
	m_get_status_time = 0;

	initNode();

	//Test
	pthread_t test_thread;
	pthread_create(&test_thread, NULL, hccb_navigation_test::thread_test, this);
}

hccb_navigation_test::~hccb_navigation_test()
{

}

void hccb_navigation_test::initNode()
{
	m_sub_status = m_nodeHandle.subscribe("/hccb/navigation/status", 1, &hccb_navigation_test::status_SubscribeCallBack, this);
	m_pub_goal = this->m_nodeHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>("/hccb/navigation/goal", 1000);
	m_pub_stop = this->m_nodeHandle.advertise<std_msgs::Empty>("/hccb/navigation/cancel", 1000);
}

//ROS Function
void hccb_navigation_test::status_SubscribeCallBack(hccb_msgs::NavigationStatus input)
{
	if(input.status == input.STATUS_MOVING)
	{
		if(m_go_and_stop == true)
		{
			stop();
			cout << "Call Stop" << endl;
			m_go_and_stop = false;
		}
		cout << "Status : MOVING" << endl;
	}

	else if(input.status == input.STATUS_FINISHED) cout << "Status : FINISHED" << endl;
	else if(input.status == input.STATUS_PLANNING_FAILED) cout << "Status : PLANNING_FAILED" << endl;
	else if(input.status == input.STATUS_CANCELED) cout << "Status : CANCELED" << endl;
	else if(input.status == input.STATUS_PAUSED) cout << "Status : PAUSED" << endl;
	else cout << "Status : UNKNOWN" << endl;
}

void hccb_navigation_test::pose_SubscribeCallBack(geometry_msgs::PoseWithCovarianceStamped input)
{
	m_robot_pose = input;
}

void hccb_navigation_test::gotogoal(int input)
{
	double goal_x, goal_y, goal_angle;
	stringstream goal_data_x, goal_data_y, goal_data_angle;
	goal_data_x<< "Goal"<<input<<"_X";
	goal_data_y<< "Goal"<<input<<"_Y";
	goal_data_angle<< "Goal"<<input<<"_ANGLE";

	goal_x = Config::getInstance()->getParamValueDouble(goal_data_x.str().c_str());
	goal_y = Config::getInstance()->getParamValueDouble(goal_data_y.str().c_str());
	goal_angle = Config::getInstance()->getParamValueDouble(goal_data_angle.str().c_str());
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(goal_angle / 57.2957795131);

	geometry_msgs::PoseWithCovarianceStamped msg;
	msg.pose.pose.position.x = goal_x;
	msg.pose.pose.position.y = goal_y;
	msg.pose.pose.position.z = 0;
	msg.pose.pose.orientation = odom_quat;
	m_pub_goal.publish(msg);

	printf("Goal Pose X : %0.3lf\tY : %0.3lf\tAngle : %0.3lf\n", goal_x, goal_y, goal_angle);
}

void hccb_navigation_test::stop()
{
	std_msgs::Empty msg;
	m_pub_stop.publish(msg);
	printf("Navigation Stop!\n");
}

//Test
void* hccb_navigation_test::thread_test(void* arg)
{
	((hccb_navigation_test*)(arg))->test();
	return 0;
}
void hccb_navigation_test::test()
{
	while(1)
	{
		int select_number;
		cout << "Select Your Service or Message!" << std::endl;
		cout << "0. Stop" << std::endl;
		cout << "1. Go To Goal 1" << std::endl;
		cout << "2. Go To Goal 2" << std::endl;
		cout << "3. Go To Goal 3" << std::endl;
		cout << "4. Go To Goal 4" << std::endl;
		cout << "5. Go To Goal 5" << std::endl;
		cout << "6. Go To Goal 6" << std::endl;
		cout << "7. Go To Goal 7" << std::endl;
		cout << "8. Go To Goal 8" << std::endl;
		cout << "9. Go To Goal 9" << std::endl;
		cout << "10.Go & Stop!" << std::endl;
		cout << "11. Exit" << std::endl;
		cin >> select_number;


		if(select_number == 0)	stop();

		else if(select_number > 0 && select_number < 10) gotogoal(select_number);

		else if(select_number == 10) m_go_and_stop = true;

		else if(select_number == 11)	break;
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "silbot_navigation_test");

	hccb_navigation_test GGG;
	
	ros::spin();
	return 0;
}
