#include <nui_tracker.h>
#include <cstdlib>
#include <cstring>
#include <iostream>

using namespace tdv::nuitrack;

nui_tracker::nui_tracker()
{
	init();

	//Data
	m_nui_data = NULL;
	if((int) m_user_data.size() > 0) m_user_data.clear();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	this->m_cloud = cloud;
	this->m_cloud->points.resize(m_width * m_height);
	this->m_cloud->width = (m_width * m_height);
	this->m_cloud->height = 1;

	//nui start
	pthread_t thread_nui;
	pthread_create(&thread_nui, NULL, nui_tracker::thread_nui_start, this);

}

nui_tracker::~nui_tracker()
{
	try
	{
		Nuitrack::release();
	}
	catch (const Exception& e)
	{
		// Do nothing
	}
}

void nui_tracker::init(const std::string& config)
{
	// Initialize Nuitrack first, then create Nuitrack modules
	try
	{
		Nuitrack::init(config);
	}
	catch (const Exception& e)
	{
		std::cerr << "Can not initialize Nuitrack (ExceptionType: " << e.type() << ")" << std::endl;
		exit(EXIT_FAILURE);
	}
	
	// Create all required Nuitrack modules

	m_depthSensor = DepthSensor::create();
	// Bind to event new frame
	m_depthSensor->connectOnNewFrame(std::bind(&nui_tracker::onNewDepthFrame, this, std::placeholders::_1));

	
	m_outputMode = m_depthSensor->getOutputMode();
	m_width = m_outputMode.xres;
	m_height = m_outputMode.yres;


	m_colorSensor = ColorSensor::create();
	// Bind to event new frame
	m_colorSensor->connectOnNewFrame(std::bind(&nui_tracker::onNewRGBFrame, this, std::placeholders::_1));
	

	m_userTracker = UserTracker::create();
	// Bind to event update user tracker
	m_userTracker->connectOnUpdate(std::bind(&nui_tracker::onUserUpdate, this, std::placeholders::_1));

	m_skeletonTracker = SkeletonTracker::create();
	// Bind to event update skeleton tracker
	m_skeletonTracker->connectOnUpdate(std::bind(&nui_tracker::onSkeletonUpdate, this, std::placeholders::_1));

	printf("Init End\n");
}

bool nui_tracker::update()
{
	if (!m_isInitialized)
	{
		// When Nuitrack modules are created, we need to call Nuitrack::run() to start processing all modules
		try
		{
			Nuitrack::run();
		}
		catch (const Exception& e)
		{
			std::cerr << "Can not start Nuitrack (ExceptionType: " << e.type() << ")" << std::endl;
			release();
			exit(EXIT_FAILURE);
		}

		m_isInitialized = true;
	}

	try
	{
		// Wait and update Nuitrack data
		Nuitrack::waitUpdate(m_skeletonTracker);
	}

	catch (const LicenseNotAcquiredException& e)
	{
		// Update failed, negative result
		std::cerr << "LicenseNotAcquired exception (ExceptionType: " << e.type() << ")" << std::endl;
		return false;
	}
	catch (const Exception& e)
	{
		// Update failed, negative result
		std::cerr << "Nuitrack update failed (ExceptionType: " << e.type() << ")" << std::endl;
		return false;
	}
	
	return true;
}

void nui_tracker::release()
{
	// Release Nuitrack and remove all modules
	try
	{
		Nuitrack::release();
	}
	catch (const Exception& e)
	{
		std::cerr << "Nuitrack release failed (ExceptionType: " << e.type() << ")" << std::endl;
	}

	m_isInitialized = false;
}

// Copy depth frame data, received from Nuitrack, to texture to visualize
void nui_tracker::onNewDepthFrame(DepthFrame::Ptr frame)
{
	const uint16_t* depthPtr = frame->getData();

	int wStep = m_width / frame->getCols();
	int hStep = m_height / frame->getRows();
	int nextVerticalBorder = hStep;

	for (size_t i = 0; i < m_height; ++i)
	{
		if (i == nextVerticalBorder)
		{
			nextVerticalBorder += hStep;
			depthPtr += frame->getCols();
		}

		int col = 0;
		int nextHorizontalBorder = wStep;
		uint16_t depthValue = *depthPtr >> 5;

		for (size_t j = 0; j < m_width; ++j)
		{
			int index = i * m_width + j;
			Vector3 point = m_depthSensor->convertProjToRealCoords(i, j, depthPtr[i,j]);

			m_cloud->points[index].x = point.z/1000;
			m_cloud->points[index].y = point.y/1000;
			m_cloud->points[index].z = -point.x/1000;

			//if(depthPtr[i,j] > 0 && depthPtr[i,j] < 2000)	printf("[%d] Ori_Z : %0.3f\tCon : %0.3f\n", index, (float) depthPtr[i,j], point.z);
		}
	}

	if(m_nui_data != NULL) m_nui_data->get_depth_xyz(m_cloud);
}

// Copy color frame data, received from Nuitrack, to texture to visualize
void nui_tracker::onNewRGBFrame(RGBFrame::Ptr frame)
{
	const tdv::nuitrack::Color3* colorPtr = frame->getData();

	int wStep = m_width / frame->getCols();
	int hStep = m_height / frame->getRows();
	wStep = (wStep > 0 ? wStep : 1);
	hStep = (hStep > 0 ? hStep : 1);

	int wStepInv = frame->getCols() / m_width;
	int hStepInv = frame->getRows() / m_height;
	wStepInv = (wStepInv > 0 ? wStepInv : 1);
	hStepInv = (hStepInv > 0 ? hStepInv : 1);

	int nextVerticalBorder = hStep;

	for (size_t i = 0; i < m_height; ++i)
	{
		if (i == nextVerticalBorder)
		{
			nextVerticalBorder += hStep;
			colorPtr += hStepInv * frame->getCols();
		}

		int col = 0;
		int nextHorizontalBorder = wStep;

		for (size_t j = 0; j < m_width; j++)
		{
			if (j == nextHorizontalBorder)
			{
				col += wStepInv;
				nextHorizontalBorder += wStep;
			}

			//texturePtr[0] = (colorPtr + col)->blue;
			//texturePtr[1] = (colorPtr + col)->green;
			//texturePtr[2] = (colorPtr + col)->red;
		}
	}
}
// Colorize user segments using Nuitrack User Tracker data
void nui_tracker::onUserUpdate(UserFrame::Ptr frame)
{
	
}

// Prepare visualization of skeletons, received from Nuitrack
void nui_tracker::onSkeletonUpdate(SkeletonData::Ptr userSkeletons)
{
	//printf("Skeleton\n");
	UserData temp_user;
	m_user_data.clear();
	auto skeletons = userSkeletons->getSkeletons();

	int count = 0;
	for (auto skeleton: skeletons)
	{
		temp_user = update_body(skeleton.joints);
		if(temp_user.flag_SkeletonData == true) m_user_data.push_back(temp_user);
		count++;
	}

	//printf("Count %d\n", count);
	if(m_nui_data != NULL) m_nui_data->get_skeleton(m_user_data);
}

UserData nui_tracker::update_body(const std::vector<Joint>& joints)
{
	UserData temp_user;
	temp_user.init();

	printf("User Confidence : %f\n", joints[JOINT_TORSO].confidence);
	if(joints[JOINT_TORSO].confidence > 0.15)
	{
		temp_user.flag_SkeletonData = true;
		if(joints[JOINT_HEAD].confidence > 0.15)
		{
			temp_user.flag_head =true;
			temp_user.head.x = joints[JOINT_HEAD].real.z/1000;
			temp_user.head.y = -joints[JOINT_HEAD].real.x/1000;
			temp_user.head.z = joints[JOINT_HEAD].real.y/1000;
		}

		if(joints[JOINT_NECK].confidence > 0.15)
		{
			temp_user.flag_neck =true;
			temp_user.neck.x = joints[JOINT_NECK].real.z/1000;
			temp_user.neck.y = -joints[JOINT_NECK].real.x/1000;
			temp_user.neck.z = joints[JOINT_NECK].real.y/1000;
		}

		if(joints[JOINT_TORSO].confidence > 0.15)
		{
			temp_user.flag_torso = true;
			temp_user.torso.x = joints[JOINT_TORSO].real.z/1000;
			temp_user.torso.y = -joints[JOINT_TORSO].real.x/1000;
			temp_user.torso.z = joints[JOINT_TORSO].real.y/1000;
		}

		if(joints[JOINT_LEFT_SHOULDER].confidence > 0.15)
		{
			temp_user.flag_left_shoulder =true;
			temp_user.left_shoulder.x = joints[JOINT_LEFT_SHOULDER].real.z/1000;
			temp_user.left_shoulder.y = -joints[JOINT_LEFT_SHOULDER].real.x/1000;
			temp_user.left_shoulder.z = joints[JOINT_LEFT_SHOULDER].real.y/1000;
		}

		if(joints[JOINT_LEFT_ELBOW].confidence > 0.15)
		{
			temp_user.flag_left_elbow =true;
			temp_user.left_elbow.x = joints[JOINT_LEFT_ELBOW].real.z/1000;
			temp_user.left_elbow.y = -joints[JOINT_LEFT_ELBOW].real.x/1000;
			temp_user.left_elbow.z = joints[JOINT_LEFT_ELBOW].real.y/1000;
		}

		if(joints[JOINT_LEFT_HAND].confidence > 0.15)
		{
			temp_user.flag_left_hand =true;
			temp_user.left_hand.x = joints[JOINT_LEFT_HAND].real.z/1000;
			temp_user.left_hand.y = -joints[JOINT_LEFT_HAND].real.x/1000;
			temp_user.left_hand.z = joints[JOINT_LEFT_HAND].real.y/1000;
		}

		if(joints[JOINT_RIGHT_SHOULDER].confidence > 0.15)
		{
			temp_user.flag_right_shoulder =true;
			temp_user.right_shoulder.x = joints[JOINT_RIGHT_SHOULDER].real.z/1000;
			temp_user.right_shoulder.y = -joints[JOINT_RIGHT_SHOULDER].real.x/1000;
			temp_user.right_shoulder.z = joints[JOINT_RIGHT_SHOULDER].real.y/1000;
		}

		if(joints[JOINT_RIGHT_ELBOW].confidence > 0.15)
		{
			temp_user.flag_right_elbow =true;
			temp_user.right_elbow.x = joints[JOINT_RIGHT_ELBOW].real.z/1000;
			temp_user.right_elbow.y = -joints[JOINT_RIGHT_ELBOW].real.x/1000;
			temp_user.right_elbow.z = joints[JOINT_RIGHT_ELBOW].real.y/1000;
		}

		if(joints[JOINT_RIGHT_HAND].confidence > 0.15)
		{
			temp_user.flag_right_hand =true;
			temp_user.right_hand.x = joints[JOINT_RIGHT_HAND].real.z/1000;
			temp_user.right_hand.y = -joints[JOINT_RIGHT_HAND].real.x/1000;
			temp_user.right_hand.z = joints[JOINT_RIGHT_HAND].real.y/1000;
		}

		if(joints[JOINT_LEFT_HIP].confidence > 0.15)
		{
			temp_user.flag_left_hip =true;
			temp_user.left_hip.x = joints[JOINT_LEFT_HIP].real.z/1000;
			temp_user.left_hip.y = -joints[JOINT_LEFT_HIP].real.x/1000;
			temp_user.left_hip.z = joints[JOINT_LEFT_HIP].real.y/1000;
		}

		if(joints[JOINT_LEFT_KNEE].confidence > 0.15)
		{
			temp_user.flag_left_knee =true;
			temp_user.left_knee.x = joints[JOINT_LEFT_KNEE].real.z/1000;
			temp_user.left_knee.y = -joints[JOINT_LEFT_KNEE].real.x/1000;
			temp_user.left_knee.z = joints[JOINT_LEFT_KNEE].real.y/1000;
		}

		if(joints[JOINT_LEFT_ANKLE].confidence > 0.15)
		{
			temp_user.flag_left_foot =true;
			temp_user.left_foot.x = joints[JOINT_LEFT_ANKLE].real.z/1000;
			temp_user.left_foot.y = -joints[JOINT_LEFT_ANKLE].real.x/1000;
			temp_user.left_foot.z = joints[JOINT_LEFT_ANKLE].real.y/1000;
		}

		if(joints[JOINT_RIGHT_HIP].confidence > 0.15)
		{
			temp_user.flag_right_hip =true;
			temp_user.right_hip.x = joints[JOINT_RIGHT_HIP].real.z/1000;
			temp_user.right_hip.y = -joints[JOINT_RIGHT_HIP].real.x/1000;
			temp_user.right_hip.z = joints[JOINT_RIGHT_HIP].real.y/1000;
		}

		if(joints[JOINT_RIGHT_KNEE].confidence > 0.15)
		{
			temp_user.flag_right_knee =true;
			temp_user.right_knee.x = joints[JOINT_RIGHT_KNEE].real.z/1000;
			temp_user.right_knee.y = -joints[JOINT_RIGHT_KNEE].real.x/1000;
			temp_user.right_knee.z = joints[JOINT_RIGHT_KNEE].real.y/1000;
		}

		if(joints[JOINT_RIGHT_ANKLE].confidence > 0.15)
		{
			temp_user.flag_right_foot =true;
			temp_user.right_foot.x = joints[JOINT_RIGHT_ANKLE].real.z/1000;
			temp_user.right_foot.y = -joints[JOINT_RIGHT_ANKLE].real.x/1000;
			temp_user.right_foot.z = joints[JOINT_RIGHT_ANKLE].real.y/1000;
		}
	}

	else temp_user.flag_SkeletonData = false;

	return temp_user;
}

//Thread
void* nui_tracker::thread_nui_start(void* arg)
{
	((nui_tracker*)(arg))->nui_start();

	return 0;
}

void nui_tracker::nui_start()
{
	printf("Nui Start!\n");

	while(1)
	{
		update();
		ros::Duration(0.05).sleep();
	}
}

//Listener
void nui_tracker::set_nui_Listener(Listener* listener)
{
	this->m_nui_data = listener;
}

