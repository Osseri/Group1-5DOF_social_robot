#include <robocare_frame.h>

robocare_frame::robocare_frame()
{
	//Sensor & Wheel
	m_sub_odom = m_nodeHandle.subscribe("/odom", 1, &robocare_frame::odom_SubscribeCallBack, this);
}

void robocare_frame::odom_SubscribeCallBack(const nav_msgs::Odometry::ConstPtr& input)
{
	static tf::TransformBroadcaster tf_broadcaster;
	double trans_x, trans_y;
	tf::Transform transform;
	if(input->pose.pose.position.x < 0.01 && input->pose.pose.position.x > -0.01) trans_x = 0.01;
	else trans_x = input->pose.pose.position.x;
	if(input->pose.pose.position.y < 0.01 && input->pose.pose.position.y > -0.01) trans_y = 0.01;
	else trans_y = input->pose.pose.position.y;

	transform.setOrigin( tf::Vector3(input->pose.pose.position.x, input->pose.pose.position.y, 0.0) );
	transform.setRotation(tf::Quaternion(input->pose.pose.orientation.x, input->pose.pose.orientation.y, input->pose.pose.orientation.z, input->pose.pose.orientation.w));
	tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
}

robocare_frame::~robocare_frame()
{

}

int main(int argc, char** argv)
{
	ros::init (argc, argv, "robocare_frame");

	robocare_frame GGG;

	ros::spin();
	return (0);
}
