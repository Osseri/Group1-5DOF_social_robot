//EUCLIDEAN_SEGMENTATION V1.05(Plane_Detection ADD)
//EUCLIDEAN_SEGMENTATION V1.06(Refactoring 3/8)

#ifndef BKK_PCL_TOOL
#define BKK_PCL_TOOL

//PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>

//Tool Class
#include "pcl_tools.h"
#include "tool_class.h"

#define POINT_FILTER_LEAF_SIZE 0.01
#define PLANE_FILTER_LEAF_SIZE 0.07

using namespace std;

class pcl_tool
{

public :
	pcl_tool();
	virtual ~pcl_tool();

	//VoxelGrid(Point Filtering)
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_filtering(pcl::PointCloud<pcl::PointXYZ>::Ptr input);

	//Plane Segmentation
	vector<eun_u::Object3D> plane_extract(pcl::PointCloud<pcl::PointXYZ>::Ptr& input, int num_plane);

	//Euclidean Segmentation
	vector<eun_u::Object3D> euclidean_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& input);

	//Region growing segmentation
	vector<eun_u::Object3D> rg_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& input);

private :
	//Tool Class
	eun_u::tool_class mat_cal;

	//PCL
	pcl::search::KdTree<pcl::PointXYZ>::Ptr m_Kdtree;								//KdTree
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> m_normal_estimator;	//Normals Estimator
	pcl::VoxelGrid<pcl::PointXYZ> m_vg;							//Filtering
	pcl::ExtractIndices<pcl::PointXYZ> m_extract;				//Extract Point
	pcl::SACSegmentation<pcl::PointXYZ> m_seg_plane;			//Extract Plane
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> m_ec;		//Extract Object by EuclideanCluster
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> m_reg;	//Extract Object by RegionGrowing

};

#endif
