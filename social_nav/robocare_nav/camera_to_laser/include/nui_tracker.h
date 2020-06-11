#ifndef NUITRACK_H_
#define NUITRACK_H_

#include <nuitrack/Nuitrack.h>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>

//Pcl
#include <tool_class.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

typedef struct UserData
{
	bool flag_SkeletonData;
	bool flag_head;
	bool flag_neck;
	bool flag_left_shoulder;
	bool flag_left_elbow;
	bool flag_left_hand;
	bool flag_right_shoulder;
	bool flag_right_elbow;
	bool flag_right_hand;
	bool flag_torso;
	bool flag_left_hip;
	bool flag_left_knee;
	bool flag_left_foot;
	bool flag_right_hip;
	bool flag_right_knee;
	bool flag_right_foot;
	eun_u::Point3D head;
	eun_u::Point3D neck;
	eun_u::Point3D left_shoulder;
	eun_u::Point3D left_elbow;
	eun_u::Point3D left_hand;
	eun_u::Point3D right_shoulder;
	eun_u::Point3D right_elbow;
	eun_u::Point3D right_hand;
	eun_u::Point3D torso;
	eun_u::Point3D left_hip;
	eun_u::Point3D left_knee;
	eun_u::Point3D left_foot;
	eun_u::Point3D right_hip;
	eun_u::Point3D right_knee;
	eun_u::Point3D right_foot;

	UserData()
	{
		init();
	}

	~UserData()
	{

	}

	void init()
	{
		flag_SkeletonData = false;
		flag_head = false;
		flag_neck = false;
		flag_left_shoulder = false;
		flag_left_elbow = false;
		flag_left_hand = false;
		flag_right_shoulder = false;
		flag_right_elbow = false;
		flag_right_hand = false;
		flag_torso = false;
		flag_left_hip = false;
		flag_left_knee = false;
		flag_left_foot = false;
		flag_right_hip = false;
		flag_right_knee = false;
		flag_right_foot = false;
		head.init();
		neck.init();
		left_shoulder.init();
		left_elbow.init();
		left_hand.init();
		right_shoulder.init();
		right_elbow.init();
		right_hand.init();
		torso.init();
		left_hip.init();
		left_knee.init();
		left_foot.init();
		right_hip.init();
		right_knee.init();
		right_foot.init();
	}
}UserData;

class Listener
{
public:
	Listener()	{}
	virtual ~Listener() {}
	virtual void get_depth_xyz(pcl::PointCloud<pcl::PointXYZ>::Ptr points) = 0;
	virtual void get_skeleton(vector<UserData> user_data) = 0;
};

class nui_tracker
{
public:
	nui_tracker();
	~nui_tracker();
	
	// Initialize sample: initialize Nuitrack, create all required modules,
	// register callbacks and start Nuitrack
	void init(const std::string& config = "");
	
	// Update the depth map, tracking and gesture recognition data,
	// then redraw the view
	bool update();
	
	// Release all sample resources
	void release();

private:
	bool m_isInitialized;
	int m_width, m_height;

	tdv::nuitrack::OutputMode m_outputMode;
	tdv::nuitrack::DepthSensor::Ptr m_depthSensor;
	tdv::nuitrack::ColorSensor::Ptr m_colorSensor;
	tdv::nuitrack::UserTracker::Ptr m_userTracker;
	tdv::nuitrack::SkeletonTracker::Ptr m_skeletonTracker;


	//Nuitrack callbacks
	void onNewDepthFrame(tdv::nuitrack::DepthFrame::Ptr frame);
	void onNewRGBFrame(tdv::nuitrack::RGBFrame::Ptr frame);
	void onUserUpdate(tdv::nuitrack::UserFrame::Ptr frame);
	void onSkeletonUpdate(tdv::nuitrack::SkeletonData::Ptr userSkeletons);
	UserData update_body(const std::vector<tdv::nuitrack::Joint>& joints);

	//Thread
	static void* thread_nui_start(void* arg);
	void nui_start();


//	//Listener
	Listener* m_nui_data;
	vector<UserData> m_user_data;
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;

public:
	void set_nui_Listener(Listener* listener);

};

#endif /* NUITRACKGLSAMPLE_H_ */
