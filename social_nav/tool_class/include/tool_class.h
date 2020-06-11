//TOOL CLASS V1.07
//TOOL CLASS V1.08(add mean and median double)

#ifndef BKK_TOOL_CLASS
#define BKK_TOOL_CLASS

#include <iostream>
#include <string>
#include <math.h>
#include <vector>
#include <stdio.h>
#include <algorithm>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//Ros Msgs
#include <sensor_msgs/LaserScan.h>

using namespace std;
using namespace Eigen;

namespace eun_u {

#define ERROR	1
#define NOT_ERROR 2
#define RADIAN_TO_DEGREE 57.2957795131
#define DEGREE_TO_RADIAN 0.01745329252

//Modeling
#define MIN_PLANE_SIZE 0.01
#define SORTING_SIZE 0.02

typedef struct Bounds3D
{
	double min_x;
	double max_x;
	double min_y;
	double max_y;
	double min_z;
	double max_z;

	Bounds3D()
	{
		init();
	}
	~Bounds3D()
	{

	}

	void init()
	{
		min_x = 1000000;
		min_y = 1000000;
		min_z = 1000000;
		max_x = -1000000;
		max_y = -1000000;
		max_z = -1000000;
	}
}Bounds3D;

typedef struct Image_resolution
{
	int image_width;
	int image_height;

	Image_resolution()
	{
		init();
	}
	~Image_resolution()
	{

	}

	void init()
	{
		image_width = 0;
		image_height = 0;
	}
}Image_resolution;

typedef struct PointUV
{
	int u;
	int v;

	PointUV()
	{
		init();
	}
	~PointUV()
	{

	}

	void init()
	{
		u = 0;
		v = 0;
	}
}PointUV;

typedef struct Color
{
	double r;
	double g;
	double b;

	Color()
	{
		init();
	}
	~Color()
	{

	}

	void init()
	{
		r = 0;
		g = 0;
		b = 0;
	}
}Color;

typedef struct Point2D
{
	double x;
	double y;

	Point2D()
	{
		init();
	}
	~Point2D()
	{

	}

	void init()
	{
		x = 0;
		y = 0;
	}
}Point2D;

typedef struct Point3D
{
	double x;
	double y;
	double z;

	Point3D()
	{
		init();
	}
	~Point3D()
	{

	}

	void init()
	{
		x = 0;
		y = 0;
		z = 0;
	}
}Point3D;

typedef struct Vector3D
{
	double X;
	double Y;
	double Z;
	Vector3D()
	{
		init();
	}
	~Vector3D()
	{

	}

	void init()
	{
		X = 0;
		Y = 0;
		Z = 0;
	}
}Vector3D;

typedef struct Object3D
{
	double width;
	double height;

	//XYZ 순서로 변환
	double angle_x;
	double angle_y;
	double angle_z;
	Point3D center_position;
	Point3D mean_center;
	Point3D median_center;
	Point3D size_center;

	double eigen_value[3];
	eun_u::Vector3D eigen_vector[3];

	vector<Point3D> points;

	Object3D()
	{
		init();
	}
	~Object3D()
	{
		if((int) points.size() > 0) points.clear();
	}

	void init()
	{
		width = 0;
		height = 0;
		angle_x = 0;
		angle_y = 0;
		angle_z = 0;

		center_position.init();
		mean_center.init();
		median_center.init();
		size_center.init();

		memset(&eigen_value, 0, sizeof(double) * 3);
		memset(&eigen_vector, 0, sizeof(eun_u::Vector3D) * 3);
		if((int) points.size() > 0) points.clear();
	}
}Object3D;

typedef struct point_box
{
	vector<Point3D> points;
	point_box()
	{
		init();
	}
	~point_box()
	{

	}

	void init()
	{
		if((int) points.size() > 0) points.clear();
	}
}point_box;

typedef struct Plane3D
{
	double width;
	double height;
	double equ_a;
	double equ_b;
	double equ_c;
	double equ_d;
	double angle_x;
	double angle_y;
	double angle_z;

	Vector3D vec_x;
	Vector3D vec_y;
	Vector3D vec_z;
	Point3D mean_center;
	Point3D median_center;
	Point3D size_center;
	Point3D octagon_poins[8];
	vector<Point3D> points;

	Plane3D()
	{
		init();
	}
	~Plane3D()
	{
		if((int) points.size() > 0) points.clear();
	}

	void init()
	{
		equ_a = 0;
		equ_b = 0;
		equ_c = 0;
		equ_d = 0;
		angle_x = 0;
		angle_y = 0;
		angle_z = 0;
		vec_x.init();
		vec_y.init();
		vec_z.init();

		mean_center.init();
		median_center.init();
		size_center.init();
		memset(octagon_poins, 0, sizeof(Point3D) * 8);
		if((int) points.size() > 0) points.clear();
	}
}Plane3D;

typedef struct Line2D	//y = bx +a;
{
	double a;
	double b;
	double distance;
	Point2D center;
	vector<Point2D> points;
	Line2D()
	{
		init();
	}
	~Line2D()
	{
		if((int) points.size() >0) points.clear();
	}

	void init()
	{
		a = 0;
		b = 0;
		distance = 0;
		center.init();
		if((int) points.size() > 0) points.clear();
	}
}Line2D;

typedef struct Line3D
{
	Vector3D direction_vector;
	Point3D mean_point;
	vector<Point3D> points;

	Line3D()
	{
		init();
	}
	~Line3D()
	{
		if((int) points.size() > 0) points.clear();
	}

	void init()
	{
		direction_vector.X = 0;
		direction_vector.Y = 0;
		direction_vector.Z = 0;
		mean_point.x = 0;
		mean_point.y = 0;
		mean_point.z = 0;
		if((int) points.size() > 0) points.clear();
	}
}Line3D;

typedef struct People3D
{
	Point3D center_position;
	double height;

	People3D()
	{
		init();
	}

	~People3D()
	{

	}

	void init()
	{
		center_position.x = 0;
		center_position.y = 0;
		center_position.z = 0;
		height = 0;
	}
}People3D;

typedef struct Tracking3D
{
	int id;
	Point3D center_position;
	double size_x;
	double size_y;
	double size_z;

	Tracking3D()
	{
		init();

	}

	~Tracking3D()
	{

	}

	void init()
	{
		center_position.x = 0;
		center_position.y = 0;
		center_position.z = 0;
		id = -1;
		size_x = 0;
		size_y = 0;
		size_z = 0;
	}
}Tracking3D;

class tool_class
{
public:
	tool_class();
	virtual ~tool_class();

	//mean double and median double
	double cal_meanDouble(vector <double> input);
	double cal_medianDouble(vector <double> input);

	//mean point and median point
	Point3D cal_meanPoint3D(vector <Point3D> input);
	Point3D cal_medianPoint3D(vector <Point3D> input);
	PointUV cal_meanPointUV(vector <PointUV> input);
	PointUV cal_medianPointUV(vector <PointUV> input);

	//Variable
	double cal_variablePoint3D(vector <Point3D> input);

	//Fitting Line2D
	Line2D fit_line2D(vector<Point2D> points);
	Line2D fit_line2D_LeastSquare(vector<Point2D> points); //y = bx + a
	void cal_2Dline_error(vector<Point2D> points, Line2D line, double* output); //[0] aver, [1]min, [2]max

	//Fitting Line3D
	Line3D fit_line3D(vector<Point3D> input);

	//calculate min and max value
	bool cal_MinAndMax_double(vector<double>& input_data, double& out_min, double& out_max);

	//Distance between PointUV and PointUV
	double distance_BetweenPointUVAndPointUV(PointUV& input_point1, PointUV& input_point2);

	//Distance between Point2D and Point2D
	double distance_BetweenPoint2DAndPoint2D(Point2D& input_point1, Point2D& input_point2);

	//Distance between Point3D and Point3D
	double distance_BetweenPoint3DAndPoint3D(Point3D& input_point1, Point3D& input_point2);

	//Distance between Line2D and Point2D
	double distance_BetweenLine2DAndPoint2D(Line2D& input_line, Point2D& input_Point);

	//Distance between ZeroPoint2D and Point2D
	double distance_BetweenZeroPointAndPoint2D(Point2D& input_Point);

	//Distance between Line3D and Point3D
	double distance_BetweenLine3DAndPoint3D(Line3D& input_line, Point3D& input_Point);

	//Distance between ZeroPoint3D and Point3D
	double distance_BetweenZeroPointAndPoint3D(Point3D& input_Point);

	//Distance between Plane and Point3D
	double distance_BetweenPlaneAndPoint3D(Plane3D& input_plane, Point3D& input_point);

	//Make the unit vector
	Vector3D make_unitvector(Vector3D input);

	//Vector DotProduct and CrossProduct(벡터의 내적, 외적)
	double DotProduct(Vector3D v1, Vector3D v2);
	Vector3D CrossProduct(Vector3D start, Vector3D dest);

	//Angle between A and B vector
	double angle_BetweenAandBvector(Vector3D A, Vector3D B);

	//Angle between Axis X, Y, Z and A vector
	double angle_BetweenAxisXandAvector(Vector3D A);
	double angle_BetweenAxisYandAvector(Vector3D A);
	double angle_BetweenAxisZandAvector(Vector3D A);

	//Rotation Vector
	Vector3D RotatedVectorAxisX(double Degree, Vector3D vec);
	Vector3D RotatedVectorAxisY(double Degree, Vector3D vec);
	Vector3D RotatedVectorAxisZ(double Degree, Vector3D vec);

	//Rotation Point2D
	Point2D RotatedPoint2D(double Degree, Point2D point);

	//Rotation Point3D
	Point3D RotatedPointAxisX(double Degree, Point3D point);
	Point3D RotatedPointAxisY(double Degree, Point3D point);
	Point3D RotatedPointAxisZ(double Degree, Point3D point);
	Point3D RotatedPointMatrix(double* rmat, Point3D point);

	//Range to Point
	Point2D convertRangeToPoint(double range, double angle);
	void convertRangeToPoints(sensor_msgs::LaserScan input, vector<Point2D>& output);

	//Translation Using TF
	Point2D TranslationPointByTF(geometry_msgs::TransformStamped tf_data, Point2D input);
	void TranslationPointsByTF(geometry_msgs::TransformStamped tf_data, vector<Point2D> input, vector<Point2D>& output);

	//Matrix Multiplication C = AB
	void multiple_matrix3x3(double* matA, double* matB, double* matC);

	//Quaternion
	Point3D QuaternionToEuler(double x, double y, double z, double w);
	void EulerToQuaternion(Point3D input, double& x, double& y, double& z, double& w);

	//Covariance and SVD
	int SVDwithCov(vector<Point3D> input, Vector3D* eigen_Vector, double* eigen_Value, Point3D& mean_point);
	int makeCovMat(vector<Point3D> input, Point3D &mean, double* out_cov);
	void CalSubMeanMatrix(vector <Point3D> input, Point3D &mean, Point3D* outmat);
	void SVD(double* covMat, Vector3D* eigen_Vector, double* eigen_Value);

	//Cal Vector Angle YZ 변환기준 -> 역변환 ZY X축 맞추기
	void cal_vector_angle(Vector3D input, double& angle_y, double& angle_z);
	void cal_pose_angle(Vector3D* input, double& angle_x, double& angle_y, double& angle_z);
	void cal_rotation_mat_XYZ(double angle_x, double angle_y, double angle_z, double* output);
	void cal_rotation_mat_ZYX(double angle_x, double angle_y, double angle_z, double* output);

	//Cal Vector Angle XYZ 변환기준 -> 역변환 ZYX 축 맞추기

	//Plane Modeling
	eun_u::Plane3D cal_plane_data(eun_u::Object3D input);
	void plane_modeling(eun_u::Plane3D& plane);
	Bounds3D plane_range(vector <Point3D> input);
	void point_sortingYZ(vector<Point3D> input, Bounds3D range, int& size, vector<Point3D>& output_points);
	void cal_octagon_point(int size, vector<Point3D> input, Point3D* output);

	//Object Modeling
	eun_u::Object3D cal_object_data(eun_u::Object3D input);
	void object_modeling(eun_u::Object3D& input);
};
}


#endif
