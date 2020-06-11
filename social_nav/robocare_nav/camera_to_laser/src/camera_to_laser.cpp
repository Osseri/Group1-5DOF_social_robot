
#include <camera_to_laser.h>

camera_to_laser::camera_to_laser()
{
	ros::NodeHandle nodeHandle;

	//Param
	nodeHandle.param("camera_to_laser/min_x", m_camera.min_x, 0.5);
	nodeHandle.param("camera_to_laser/max_x", m_camera.max_x, 3.5);
	nodeHandle.param("camera_to_laser/min_y", m_camera.min_y, -1.5);
	nodeHandle.param("camera_to_laser/max_y", m_camera.max_y, 1.5);
	nodeHandle.param("camera_to_laser/min_z", m_camera.min_z, 0.0);
	nodeHandle.param("camera_to_laser/max_z", m_camera.max_z, 1.5);

	//ros
	this->m_pub_laser = nodeHandle.advertise<sensor_msgs::LaserScan>("/astra/scan", 10);

	//CallBack
	this->m_nui_tracker.set_nui_Listener((Listener*) this);


}

camera_to_laser::~camera_to_laser()
{

}

//CallBack
void camera_to_laser::get_depth_xyz(pcl::PointCloud<pcl::PointXYZ>::Ptr points)
{
	double distance[270];
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;

	cloud_filtered = pcl.point_filtering(points);
	//printf("Before : %d\tAfter : %d\n", (int) points->points.size(), (int) cloud_filtered->points.size());
	conversion_to_laser(cloud_filtered, distance);

	pub_laser(distance);
	//printf("\n");
}

void camera_to_laser::get_skeleton(vector<UserData> user_data)
{

}

void camera_to_laser::conversion_to_laser(pcl::PointCloud<pcl::PointXYZ>::Ptr input, double* output)
{
	int angle;
	double distance;
	eun_u::Vector3D vec;
	eun_u::Point2D point;
	memset(output, 0, sizeof(double) * 270);

	for(int i = 0; i < (int) input->points.size(); i++)
	{
		if(m_camera.min_x < input->points[i].x && m_camera.max_x > input->points[i].x &&
				m_camera.min_y < input->points[i].y && m_camera.max_y > input->points[i].y &&
				m_camera.min_z < input->points[i].z && m_camera.max_z > input->points[i].z)
		{
			point.x = vec.X = input->points[i].x;
			point.y = vec.Y = input->points[i].y;
			vec.Z = 0;
			angle = (mat_cal.angle_BetweenAxisXandAvector(vec) * 3);

			if(vec.Y < 0) angle = (int) (-fabs(angle) + 135);
			else angle = (int) (fabs(angle) + 135);
			distance = mat_cal.distance_BetweenZeroPointAndPoint2D(point);

			if(angle > 0 && angle < 270 && distance != 0)
			{
				if(distance < output[angle] || output[angle] == 0)	output[angle] = distance;
			}
		}
	}
}

//Scan
void camera_to_laser::pub_laser(double* input)
{
	sensor_msgs::LaserScan out_put;
	ros::Time time = ros::Time::now();

	out_put.header.stamp = time;
	out_put.header.frame_id = "astra";
	out_put.angle_min = -45 * DEGREE_TO_RADIAN;
	out_put.angle_max = 45 * DEGREE_TO_RADIAN;
	out_put.angle_increment = DEGREE_TO_RADIAN * 0.3333333333333;
	out_put.time_increment = 0.00005;
	out_put.scan_time = ros::Time::now().toSec();
	out_put.range_min = 0.01;
	out_put.range_max = 10.0;

	for(int i = 0; i < 270; i++)
	{
		out_put.ranges.push_back(input[i]);
	}

	m_pub_laser.publish(out_put);
}

//Display
/*
void camera_to_laser::laser_display(vector<eun_u::Object3D> input)
{
	visualization_msgs::Marker points0, points1, points2;
	points0.header.frame_id = "/camera_depth_frame";
	points0.header.stamp = ros::Time::now();
	points0.ns = "Object";
	points0.action = visualization_msgs::Marker::ADD;
	points0.pose.orientation.w = 1.0;
	points0.type = visualization_msgs::Marker::POINTS;

	points0.scale.x = 0.01;
	points0.scale.y = 0.01;
	points0.scale.z = 0.01;

	points1 = points2 = points0;

	points0.id = 1;
	points1.id = 2;
	points2.id = 3;

	points0.color.r = 1.0f;
	points0.color.g = 0.0f;
	points0.color.b = 0.0f;

	points1.color.r = 0.0f;
	points1.color.g = 1.0f;
	points1.color.b = 0.0f;

	points2.color.r = 0.0f;
	points2.color.g = 0.0f;
	points2.color.b = 1.0f;

	points0.color.a = points1.color.a = points2.color.a = 1.0;

	if((int) input.size() > 0)
	{
		geometry_msgs::Point p;
		for(int i = 0; i < (int) input.size(); i++)
		{
			// POINTS markers use x and y scale for width/height respectively
			for(int j = 0; j < (int) input[i].points.size(); j++)
			{
				p.x = (float) input[i].points[j].x;
				p.y = (float) input[i].points[j].y;
				p.z = (float) input[i].points[j].z;

				if(i%3 == 0) points0.points.push_back(p);
				else if(i%3 == 1) points1.points.push_back(p);
				else points2.points.push_back(p);
			}
		}

		m_pub_object.publish(points0);
		m_pub_object.publish(points1);
		m_pub_object.publish(points2);
	}
}
*/

int main(int argc, char** argv)
{
	ros::init (argc, argv, "pcl_test");

	camera_to_laser GGG;

	ros::MultiThreadedSpinner spinner(4); // Use 4 threads
	spinner.spin();

	return (0);
}

