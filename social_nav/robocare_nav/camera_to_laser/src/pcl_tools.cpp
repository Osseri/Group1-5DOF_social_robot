//EUCLIDEAN_SEGMENTATION V1.05(Plane_Detection ADD)

#include <pcl_tools.h>
pcl_tool::pcl_tool()
{
	//KdTree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	m_Kdtree = tree;

	//Normals
	m_normal_estimator.setSearchMethod (m_Kdtree);

	//Point Filtering
	pcl::VoxelGrid<pcl::PointXYZ> m_vg;

	//Segmentation Plane
	m_seg_plane.setOptimizeCoefficients (true);
	m_seg_plane.setModelType (pcl::SACMODEL_PLANE);
	m_seg_plane.setMethodType (pcl::SAC_RANSAC);
	m_seg_plane.setMaxIterations (100);
	m_seg_plane.setDistanceThreshold (0.02);

	//Extract Object by EuclideanCluster
	m_ec.setClusterTolerance (0.3);
	m_ec.setMinClusterSize (10);
	m_ec.setMaxClusterSize (50000);
	m_ec.setSearchMethod (m_Kdtree);

	//Normals Estimator
	m_normal_estimator.setKSearch (50);

	//Extract Object by RegionGrowing
	m_reg.setMinClusterSize (50);
	m_reg.setMaxClusterSize (1000000);
	m_reg.setSearchMethod (m_Kdtree);
	m_reg.setNumberOfNeighbours (30);
	m_reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
	m_reg.setCurvatureThreshold (1.0);
}

pcl_tool::~pcl_tool()
{

}

//Point Filtering
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_tool::point_filtering(pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr return_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	m_vg.setInputCloud (input);
	m_vg.setLeafSize (POINT_FILTER_LEAF_SIZE, POINT_FILTER_LEAF_SIZE, POINT_FILTER_LEAF_SIZE);
	m_vg.filter (*return_cloud);

	return return_cloud;
}

//Plane Segmentation
vector<eun_u::Object3D> pcl_tool::plane_extract(pcl::PointCloud<pcl::PointXYZ>::Ptr& input, int num_plane)
{
	vector<eun_u::Object3D> return_object;
	if((int) return_object.size() > 0) return_object.clear();

	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

	if((int) input->points.size() > 100)
	{
		for(int i = 0; i < num_plane; i++)
		{
			printf("plane[%d] input Point num : %d\n", i, (int) input->points.size());

			m_seg_plane.setInputCloud(input);
			m_seg_plane.segment(*inliers, *coefficients);
			if (inliers->indices.size () == 0)
			{
				std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
				break;
			}

			// Extract the planar inliers from the input cloud
			m_extract.setInputCloud(input);
			m_extract.setIndices(inliers);
			m_extract.setNegative(false);
			m_extract.filter(*cloud_plane);

			// Remove the planar inliers, extract the rest
			m_extract.setNegative (true);
			m_extract.filter(*filter_cloud);
			*input = *filter_cloud;
			//printf("plane[%d] Plane num : %d\n", i, (int) cloud_plane->points.size());
			//printf("plane[%d] Remain num : %d\n", i, (int) input->points.size());

			eun_u::Point3D temp_point;
			eun_u::Object3D temp_object;
			for(int i = 0; i < (int) cloud_plane->points.size(); i++)
			{
				temp_point.x = cloud_plane->points[i].x;
				temp_point.y = cloud_plane->points[i].y;
				temp_point.z = cloud_plane->points[i].z;
				temp_object.points.push_back(temp_point);
			}

			temp_object.center_position = mat_cal.cal_medianPoint3D(temp_object.points);
			return_object.push_back(temp_object);

			if((int) input->points.size() < 100)	break;
		}
	}

	return return_object;
}

vector<eun_u::Object3D> pcl_tool::euclidean_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& input)
{
	vector<eun_u::Object3D> return_object;
	if((int) return_object.size() > 0 ) return_object.clear();

	m_Kdtree->setInputCloud(input);
	std::vector<pcl::PointIndices> cluster_indices;
	m_ec.setInputCloud (input);
	m_ec.extract (cluster_indices);

	if((int) cluster_indices.size() > 0)
	{
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			eun_u::Object3D temp_object;

			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
			{
				eun_u::Point3D temp_point;
				temp_point.x = input->points[*pit].x;
				temp_point.y = input->points[*pit].y;
				temp_point.z = input->points[*pit].z;
				temp_object.points.push_back(temp_point);
			}

			temp_object.center_position = mat_cal.cal_medianPoint3D(temp_object.points);
			return_object.push_back(temp_object);
		}
	}

	return return_object;
}

vector<eun_u::Object3D> pcl_tool::rg_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& input)
{
	vector<eun_u::Object3D> return_object;
	if((int) return_object.size() > 0 ) return_object.clear();

	pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
	m_normal_estimator.setInputCloud(input);
	m_normal_estimator.compute (*normals);

	m_reg.setInputNormals (normals);
	m_reg.setInputCloud (input);
	std::vector <pcl::PointIndices> clusters;
	m_reg.extract (clusters);

	if((int) clusters.size() > 0)
	{
		for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin (); it != clusters.end (); ++it)
		{
			eun_u::Object3D temp_object;

			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
			{
				eun_u::Point3D temp_point;
				temp_point.x = input->points[*pit].x;
				temp_point.y = input->points[*pit].y;
				temp_point.z = input->points[*pit].z;
				temp_object.points.push_back(temp_point);
			}

			temp_object.center_position = mat_cal.cal_medianPoint3D(temp_object.points);
			return_object.push_back(temp_object);
		}
	}

	return return_object;
}

