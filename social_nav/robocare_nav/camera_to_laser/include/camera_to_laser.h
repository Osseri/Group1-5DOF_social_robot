
#include "nui_tracker.h"
#include "tool_class.h"

//Ros
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

//PCL
#include <pcl_tools.h>

class camera_to_laser : public  Listener
{
public :
	camera_to_laser();
	virtual ~camera_to_laser();

private :
	pcl_tool pcl;
	eun_u::tool_class mat_cal;
	eun_u::Bounds3D m_camera;

	//Ros
	ros::Publisher m_pub_laser;

	//Callback
	nui_tracker m_nui_tracker;
	void get_depth_xyz(pcl::PointCloud<pcl::PointXYZ>::Ptr points);
	void get_skeleton(vector<UserData> user_data);
	void conversion_to_laser(pcl::PointCloud<pcl::PointXYZ>::Ptr input, double* output);

	//Scan
	void pub_laser(double* input);

	//Display
	void laser_display(vector<eun_u::Object3D> input);
};

