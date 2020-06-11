
#include "scan_fusion.h"

scan_fusion::scan_fusion()
{
	if((int) m_astra.size() > 0) m_astra.clear();
	m_sub_laser = m_nodeHandle.subscribe("/robocare/scan", 1, &scan_fusion::laser_SubscribeCallBack, this);
	m_sub_astra = m_nodeHandle.subscribe("/astra/scan", 1, &scan_fusion::astra_SubscribeCallBack, this);;
	m_pub_data = this->m_nodeHandle.advertise<sensor_msgs::LaserScan>("/fusion/scan", 1000);
}

scan_fusion::~scan_fusion()
{

}

//ROS Function
void scan_fusion::laser_SubscribeCallBack(sensor_msgs::LaserScan input)
{
	fusion(input);
}

void scan_fusion::astra_SubscribeCallBack(sensor_msgs::LaserScan input)
{
	if((int) input.ranges.size() > 0)
	{
		geometry_msgs::PointStamped astra;
		geometry_msgs::PointStamped laser;
		tf2_ros::Buffer tf2_Buffer;
		tf2_ros::TransformListener tf2_listener(tf2_Buffer);
		geometry_msgs::TransformStamped astra_to_laser;
		ros::Time past = ros::Time::now() - ros::Duration(5.0);

		try{
			astra_to_laser = tf2_Buffer.lookupTransform("laser", "astra",	ros::Time(0), ros::Duration(1.0) );
			astra.header.frame_id = "astra";

			double angle;
			eun_u::Point2D temp_axis;
			if((int) m_astra.size() > 0) m_astra.clear();

			for(int i = 0; i < input.ranges.size(); i++)
			{
				if( input.ranges[i] > 0.1)
				{
					angle = (input.angle_min + (input.angle_increment * i)) * RADIAN_TO_DEGREE;
					temp_axis = mat_cal.convertRangeToPoint(input.ranges[i], angle);
					astra.point.x = temp_axis.x;
					astra.point.y = temp_axis.y;
					astra.point.z = 0;
					tf2::doTransform(astra, laser, astra_to_laser);
					temp_axis.x = laser.point.x;
					temp_axis.y = laser.point.y;
					m_astra.push_back(temp_axis);
				}
			}
		}
		catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			if((int) m_astra.size() > 0) m_astra.clear();
			ros::Duration(1.0).sleep();
		}
	}
}

void scan_fusion::fusion(sensor_msgs::LaserScan laser)
{
	int index;
	double angle;
	double* astra_distance = NULL;
	eun_u::Vector3D vec;
	vector<eun_u::Point2D> astra_axis;
	astra_distance = new double [(int) laser.ranges.size()];

	sensor_msgs::LaserScan msg;

	if((int) astra_axis.size() > 0) astra_axis.clear();
	memset(astra_distance, 0, sizeof(double) * (int) laser.ranges.size());

	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "laser";
	msg.angle_min = laser.angle_min;
	msg.angle_max = laser.angle_max;
	msg.angle_increment = laser.angle_increment;
	msg.time_increment = laser.time_increment;
	msg.scan_time = ros::Time::now().toSec();
	msg.range_min = laser.range_min;
	msg.range_max = laser.range_max;

	printf("Num Range : %d\n", (int) laser.ranges.size());

	if((int) m_astra.size() > 0)
	{
		astra_axis = m_astra;
		for(int i = 0; i < (int) astra_axis.size(); i++)
		{
			vec.X = astra_axis[i].x;
			vec.Y = astra_axis[i].y;
			angle = mat_cal.angle_BetweenAxisXandAvector(vec) * DEGREE_TO_RADIAN;

			if(vec.Y < 0) angle = -fabs(angle);
			else angle = fabs(angle);
			index = (int) ((angle - laser.angle_min)/laser.angle_increment);
			astra_distance[index] = mat_cal.distance_BetweenZeroPointAndPoint2D(m_astra[i]);
		}

		for(int i = 0; i < (int) laser.ranges.size(); i++)
		{
			if(astra_distance[i] != 0)
			{
				if(laser.ranges[i] < astra_distance[i])	msg.ranges.push_back(laser.ranges[i]);
				else msg.ranges.push_back(astra_distance[i]);
			}

			else msg.ranges.push_back(laser.ranges[i]);
		}
	}

	else
	{
		for(int i = 0; i < (int) laser.ranges.size(); i++)	msg.ranges.push_back(laser.ranges[i]);
	}

	m_pub_data.publish(msg);

	if(astra_distance != NULL) delete [] astra_distance;
	if((int) astra_axis.size() > 0) astra_axis.clear();
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "scan_fusion");
	scan_fusion GGG;
	ros::MultiThreadedSpinner spinner(3);
	spinner.spin();
	return 0;
}
