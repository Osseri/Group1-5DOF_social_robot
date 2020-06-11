//TOOL CLASS V1.07
//TOOL CLASS V1.08(add mean and median double)

#include <tool_class.h>

using namespace eun_u;

tool_class::tool_class()
{

}

tool_class::~tool_class()
{

}

//mean point and median point
double tool_class::cal_meanDouble(vector <double> input)
{
	double mean = 0;

	if((int) input.size() <= 0)
	{
		printf("[ERROR] function 'cal_meanDouble' input is wrong!\n");
		return mean;
	}

	for(int i=0; i<(int)input.size(); i++) mean = mean + input[i];
	mean = mean / input.size();

	return mean;
}

double tool_class::cal_medianDouble(vector <double> input)
{
	double median;
	if((int) input.size() <= 0)
	{
		printf("[ERROR] function 'cal_medianDouble' input is wrong!\n");
		return median;
	}

	int num_point = (int) input.size();
	int median_index = num_point/2;

	vector<double> array;
	array = input;

	sort(array.begin(), array.end());

	median = array[median_index];

	if((int) array.size() > 0) array.clear();
	return median;
}

//mean point and median point
Point3D tool_class::cal_meanPoint3D(vector <Point3D> input)
{
	int i;
	Point3D mean;
	if((int) input.size() <= 0)
	{
		printf("[ERROR] function 'cal_meanPoint3D' input is wrong!\n");
		return mean;
	}

	mean.x = 0;	mean.y = 0; mean.z = 0;

	for(i=0; i<(int)input.size(); i++)
	{
		mean.x = mean.x + input[i].x;
		mean.y = mean.y + input[i].y;
		mean.z = mean.z + input[i].z;
	}

	mean.x = mean.x / input.size();
	mean.y = mean.y / input.size();
	mean.z = mean.z / input.size();

	return
		mean;
}

Point3D tool_class::cal_medianPoint3D(vector <Point3D> input)
{
	Point3D median_point;
	if((int) input.size() <= 0)
	{
		printf("[ERROR] function 'cal_medianPoint3D' input is wrong!\n");
		return median_point;
	}

	int num_point = (int) input.size();
	int median_index = num_point/2;
	vector<double> array_x;
	vector<double> array_y;
	vector<double> array_z;

	for(int i = 0; i < num_point; i++)
	{
		array_x.push_back(input[i].x);
		array_y.push_back(input[i].y);
		array_z.push_back(input[i].z);
	}

	sort(array_x.begin(), array_x.end());
	sort(array_y.begin(), array_y.end());
	sort(array_z.begin(), array_z.end());

	median_point.x = array_x[median_index];
	median_point.y = array_y[median_index];
	median_point.z = array_z[median_index];

	if((int) array_x.size() > 0) array_x.clear();
	if((int) array_y.size() > 0) array_y.clear();
	if((int) array_z.size() > 0) array_z.clear();

	/*
	printf("\n");
	for(int i = 0; i < num_point; i++)
		printf("Point[%d] X : %0.3lf\t Y : %0.3lf\tZ : %0.3lf\n", i, array_x[i], array_y[i], array_z[i]);
	printf("\n");
	*/

	return median_point;
}

PointUV tool_class::cal_meanPointUV(vector <PointUV> input)
{
	int i;
	PointUV mean;
	if((int) input.size() <= 0)
	{
		printf("[ERROR] function 'cal_meanPointUV' input is wrong!\n");
		return mean;
	}

	mean.u = 0;	mean.v = 0;

	for(i=0; i<(int)input.size(); i++)
	{
		mean.u = mean.u + input[i].u;
		mean.v = mean.v + input[i].v;
	}

	mean.u = mean.u / input.size();
	mean.v = mean.v / input.size();

	return	mean;
}

PointUV tool_class::cal_medianPointUV(vector <PointUV> input)
{
	PointUV median_point;
	if((int) input.size() <= 2)
	{
		printf("[ERROR] function 'cal_medianPointUV' input is wrong!\n");
		return median_point;
	}

	int num_point = (int) input.size();
	int median_index = num_point/2;
	vector<double> array_u;
	vector<double> array_v;


	for(int i = 0; i < num_point; i++)
	{
		array_u.push_back(input[i].u);
		array_v.push_back(input[i].v);
	}

	sort(array_u.begin(), array_u.end());
	sort(array_v.begin(), array_v.end());

	median_point.u = array_u[median_index];
	median_point.v = array_v[median_index];


	if((int) array_u.size() > 0) array_u.clear();
	if((int) array_v.size() > 0) array_v.clear();

	return median_point;
}

//Variable
double tool_class::cal_variablePoint3D(vector <Point3D> input)
{
	Point3D average = cal_meanPoint3D(input);
	double vari = 0;
	double distance;

	for(int i = 0; i < (int) input.size(); i++)
	{
		distance = distance_BetweenPoint3DAndPoint3D(average, input[i]);
		vari += distance;
	}

	vari = vari/input.size();
	return vari;
}

//calculate min and max value
bool tool_class::cal_MinAndMax_double(vector<double>& input_data, double& out_min, double& out_max)
{
	if((int) input_data.size() <= 0)
	{
		printf("[ERROR] function 'cal_MinAndMax' input is wrong!\n");
		return false;
	}

	vector<double> temp_data = input_data;

	sort(temp_data.begin(), temp_data.end());

	out_min = temp_data[0];
	out_max = temp_data[(int) input_data.size() - 1];

	if((int) temp_data.size() > 0 ) temp_data.clear();

	/*
	printf("\n");
	printf("Min Value : %0.3lf\tMax Value : %0.3lf\n", out_min, out_max);
	printf("\n");
	 */

	return true;
}

//Distance between PointUV and PointUV
double tool_class::distance_BetweenPointUVAndPointUV(PointUV& input_point1, PointUV& input_point2)
{
	double point_distance;

	point_distance = pow((input_point1.u - input_point2.u), 2) + pow((input_point1.v - input_point2.v), 2);
	point_distance = sqrt(point_distance);

	return point_distance;
}

//Distance between Point2D and Point2D
double tool_class::distance_BetweenPoint2DAndPoint2D(Point2D& input_point1, Point2D& input_point2)
{
	double point_distance;

	point_distance = pow((input_point1.x - input_point2.x), 2) + pow((input_point1.y - input_point2.y), 2);
	point_distance = sqrt(point_distance);

	return point_distance;
}

//Distance between ZeroPoint2D and Point2D
double tool_class::distance_BetweenZeroPointAndPoint2D(Point2D& input_Point)
{
	double point_distance;

	point_distance = pow(input_Point.x, 2) + pow(input_Point.y, 2);
	point_distance = sqrt(point_distance);

	return point_distance;
}

//Distance between Point3D and Point3D
double tool_class::distance_BetweenPoint3DAndPoint3D(Point3D& input_point1, Point3D& input_point2)
{
	double point_distance;

	point_distance = pow((input_point1.x - input_point2.x), 2) + pow((input_point1.y - input_point2.y), 2) + pow((input_point1.z - input_point2.z), 2);
	point_distance = sqrt(point_distance);

	return point_distance;
}

//Distance between ZeroPoint3D and Point3D
double tool_class::distance_BetweenZeroPointAndPoint3D(Point3D& input_Point)
{
	double point_distance;

	point_distance = pow(input_Point.x, 2) + pow(input_Point.y, 2) + pow(input_Point.z, 2);
	point_distance = sqrt(point_distance);

	return point_distance;
}

//Distance between Line2D and Point2D
double tool_class::distance_BetweenLine2DAndPoint2D(Line2D& input_line, Point2D& input_Point)
{
	double distance;
	double variance[2];

	variance[0] = input_line.b * input_Point.x - input_Point.y + input_line.a;
	variance[0] = fabs(variance[0]);
	variance[1] = pow(input_line.b,2) + 1;
	variance[1] = sqrt(variance[1]);

	distance = variance[0]/variance[1];
	return distance;
}

//Distance between Line3D and Point3D
double tool_class::distance_BetweenLine3DAndPoint3D(Line3D& input_line, Point3D& input_Point)
{
	double distance;
	double t, variance[4];
	Vector3D point_to_line;

	variance[0] = input_line.direction_vector.X*(input_Point.x - input_line.mean_point.x);
	variance[1] = input_line.direction_vector.Y*(input_Point.y - input_line.mean_point.y);
	variance[2] = input_line.direction_vector.Z*(input_Point.z - input_line.mean_point.z);
	variance[3] = pow(input_line.direction_vector.X,2) + pow(input_line.direction_vector.Y,2) + pow(input_line.direction_vector.Z,2);
	t = (variance[0] + variance[1] +variance[2])/variance[3];

	point_to_line.X = input_Point.x - (t * input_line.direction_vector.X + input_line.mean_point.x);
	point_to_line.Y = input_Point.y - (t * input_line.direction_vector.Y + input_line.mean_point.y);
	point_to_line.Z = input_Point.z - (t * input_line.direction_vector.Z + input_line.mean_point.z);

	variance[0] = pow(point_to_line.X, 2);
	variance[1] = pow(point_to_line.Y, 2);
	variance[2] = pow(point_to_line.Z, 2);

	distance = sqrt(variance[0] + variance[1] + variance[2]);
	return distance;
}


//Distance between Plane and Point3D
double tool_class::distance_BetweenPlaneAndPoint3D(Plane3D& input_plane, Point3D& input_point)
{
	double Variable0, Variable1, Variable2, Variable3;
	double return_data;

	Variable0 = input_plane.equ_a*input_point.x + input_plane.equ_b*input_point.y + input_plane.equ_c*input_point.z + input_plane.equ_d;
	Variable1 = pow(input_plane.equ_a, 2) + pow(input_plane.equ_b, 2) + pow(input_plane.equ_c, 2);
	Variable2 = fabs(Variable0);
	Variable3 = sqrt(Variable1);
	return_data = Variable2/Variable3;

	return return_data;
}

//Make the unit vector
Vector3D tool_class::make_unitvector(Vector3D input)
{
	Vector3D unit_vector = input;

	double amp, outX, outY, outZ;
	outX = pow(unit_vector.X*1000, 2);
	outY = pow(unit_vector.Y*1000, 2);
	outZ = pow(unit_vector.Z*1000, 2);

	amp = sqrt(outX + outY + outZ)/1000;

	if(amp!=0)
	{
		unit_vector.X = unit_vector.X/amp;
		unit_vector.Y = unit_vector.Y/amp;
		unit_vector.Z = unit_vector.Z/amp;
	}
	else
	{
		unit_vector.X = 0;
		unit_vector.Y = 0;
		unit_vector.Z = 0;
	}

	//printf("Amp : %lf\n", amp);
	//printf("Unit_Vector X : %0.3lf\tY : %0.3lf\tZ : %0.3lf\n", unit_vector.X, unit_vector.Y, unit_vector.Z);
	return unit_vector;
}

//Vector DotProduct and CrossProduct(벡터의 내적, 외적)
double tool_class::DotProduct(Vector3D v1, Vector3D v2)
{
	double outValue;

	outValue = v1.X*v2.X + v1.Y*v2.Y + v1.Z*v2.Z;

	return	outValue;
}

Vector3D tool_class::CrossProduct(Vector3D start, Vector3D dest)
{
	Vector3D C;
	C.X = start.Y * dest.Z - start.Z * dest.Y;
	C.Y = start.Z * dest.X - start.X * dest.Z;
	C.Z = start.X * dest.Y - start.Y * dest.X;
	return C;
}

//Angle between A and B vector
double tool_class::angle_BetweenAandBvector(Vector3D A, Vector3D B)
{
	double angle;
	Vector3D unit_A, unit_B;

	unit_A = make_unitvector(A);
	unit_B = make_unitvector(B);

	angle = acos(DotProduct(unit_A, unit_B)) * RADIAN_TO_DEGREE;

	return angle;
}

//Angle between Axis X, Y, Z and A vector
double tool_class::angle_BetweenAxisXandAvector(Vector3D A)
{
	double angle;
	Vector3D axis_X, unit_A;

	axis_X.X = 1;
	axis_X.Y = 0;
	axis_X.Z = 0;

	unit_A = make_unitvector(A);

	angle = acos(DotProduct(axis_X, unit_A)) * RADIAN_TO_DEGREE;

	return angle;
}

double tool_class::angle_BetweenAxisYandAvector(Vector3D A)
{
	double angle;
	Vector3D axis_Y, unit_A;

	axis_Y.X = 0;
	axis_Y.Y = 1;
	axis_Y.Z = 0;

	unit_A = make_unitvector(A);

	angle = acos(DotProduct(axis_Y, unit_A)) * RADIAN_TO_DEGREE;

	return angle;
}

double tool_class::angle_BetweenAxisZandAvector(Vector3D A)
{
	double angle;
	Vector3D axis_Z, unit_A;

	axis_Z.X = 0;
	axis_Z.Y = 0;
	axis_Z.Z = 1;

	unit_A = make_unitvector(A);

	angle = acos(DotProduct(axis_Z, unit_A)) * RADIAN_TO_DEGREE;

	return angle;
}

//Rotation Vector
Vector3D tool_class::RoatedVectorAxisX(double Degree, Vector3D vec)
{
	double r_mat[9];
	double angle;
	Vector3D temp_Vec;

	angle = Degree/RADIAN_TO_DEGREE;

	r_mat[0] = 1;	r_mat[1] = 0;				r_mat[2] = 0;
	r_mat[3] = 0;	r_mat[4] = cos(angle);	r_mat[5] = -sin(angle);
	r_mat[6] = 0;	r_mat[7] = sin(angle);	r_mat[8] = cos(angle);

	temp_Vec.X = vec.X * r_mat[0] + vec.Y * r_mat[1] + vec.Z * r_mat[2];
	temp_Vec.Y = vec.X * r_mat[3] + vec.Y * r_mat[4] + vec.Z * r_mat[5];
	temp_Vec.Z = vec.X * r_mat[6] + vec.Y * r_mat[7] + vec.Z * r_mat[8];

	return temp_Vec;
}

Vector3D tool_class::RoatedVectorAxisY(double Degree, Vector3D vec)
{
	double r_mat[9];
	double angle;
	Vector3D temp_Vec;

	angle = Degree/RADIAN_TO_DEGREE;

	r_mat[0] = cos(angle);	r_mat[1] = 0;	r_mat[2] = sin(angle);
	r_mat[3] = 0;				r_mat[4] = 1;	r_mat[5] = 0;
	r_mat[6] = -sin(angle);	r_mat[7] = 0;	r_mat[8] = cos(angle);

	temp_Vec.X = vec.X * r_mat[0] + vec.Y * r_mat[1] + vec.Z * r_mat[2];
	temp_Vec.Y = vec.X * r_mat[3] + vec.Y * r_mat[4] + vec.Z * r_mat[5];
	temp_Vec.Z = vec.X * r_mat[6] + vec.Y * r_mat[7] + vec.Z * r_mat[8];

	return temp_Vec;
}

Vector3D tool_class::RoatedVectorAxisZ(double Degree, Vector3D vec)
{
	double r_mat[9];
	double angle;
	Vector3D temp_Vec;

	angle = Degree/RADIAN_TO_DEGREE;

	r_mat[0] = cos(angle);	r_mat[1] = -sin(angle);	r_mat[2] = 0;
	r_mat[3] = sin(angle);	r_mat[4] = cos(angle);	r_mat[5] = 0;
	r_mat[6] = 0;				r_mat[7] = 0;				r_mat[8] = 1;

	temp_Vec.X = vec.X * r_mat[0] + vec.Y * r_mat[1] + vec.Z * r_mat[2];
	temp_Vec.Y = vec.X * r_mat[3] + vec.Y * r_mat[4] + vec.Z * r_mat[5];
	temp_Vec.Z = vec.X * r_mat[6] + vec.Y * r_mat[7] + vec.Z * r_mat[8];

	return temp_Vec;
}

//Rotation Point
Point3D tool_class::RoatedPointAxisX(double Degree, Point3D point)
{
	double r_mat[9];
	double angle;
	Point3D temp_point;

	angle = Degree/RADIAN_TO_DEGREE;

	r_mat[0] = 1;	r_mat[1] = 0;				r_mat[2] = 0;
	r_mat[3] = 0;	r_mat[4] = cos(angle);	r_mat[5] = -sin(angle);
	r_mat[6] = 0;	r_mat[7] = sin(angle);	r_mat[8] = cos(angle);

	temp_point.x = point.x * r_mat[0] + point.y * r_mat[1] + point.z * r_mat[2];
	temp_point.y = point.x * r_mat[3] + point.y * r_mat[4] + point.z * r_mat[5];
	temp_point.z = point.x * r_mat[6] + point.y * r_mat[7] + point.z * r_mat[8];

	return temp_point;
}

Point3D tool_class::RoatedPointAxisY(double Degree, Point3D point)
{
	double r_mat[9];
	double angle;
	Point3D temp_point;

	angle = Degree/RADIAN_TO_DEGREE;

	r_mat[0] = cos(angle);	r_mat[1] = 0;	r_mat[2] = sin(angle);
	r_mat[3] = 0;				r_mat[4] = 1;	r_mat[5] = 0;
	r_mat[6] = -sin(angle);	r_mat[7] = 0;	r_mat[8] = cos(angle);

	temp_point.x = point.x * r_mat[0] + point.y * r_mat[1] + point.z * r_mat[2];
	temp_point.y = point.x * r_mat[3] + point.y * r_mat[4] + point.z * r_mat[5];
	temp_point.z = point.x * r_mat[6] + point.y * r_mat[7] + point.z * r_mat[8];

	return temp_point;
}

Point3D tool_class::RoatedPointAxisZ(double Degree, Point3D point)
{
	double r_mat[9];
	double angle;
	Point3D temp_point;

	angle = Degree/RADIAN_TO_DEGREE;

	r_mat[0] = cos(angle);	r_mat[1] = -sin(angle);	r_mat[2] = 0;
	r_mat[3] = sin(angle);	r_mat[4] = cos(angle);	r_mat[5] = 0;
	r_mat[6] = 0;				r_mat[7] = 0;				r_mat[8] = 1;

	temp_point.x = point.x * r_mat[0] + point.y * r_mat[1] + point.z * r_mat[2];
	temp_point.y = point.x * r_mat[3] + point.y * r_mat[4] + point.z * r_mat[5];
	temp_point.z = point.x * r_mat[6] + point.y * r_mat[7] + point.z * r_mat[8];

	return temp_point;
}

Point3D tool_class::RoatedPointMatrix(double* rmat, Point3D point)
{
	Point3D temp_point;

	temp_point.x = point.x * rmat[0] + point.y * rmat[1] + point.z * rmat[2];
	temp_point.y = point.x * rmat[3] + point.y * rmat[4] + point.z * rmat[5];
	temp_point.z = point.x * rmat[6] + point.y * rmat[7] + point.z * rmat[8];

	return temp_point;
}

void tool_class::multiple_matrix3x3(double* matA, double* matB, double* matC)
{
	matC[0] = matA[0] * matB[0] + matA[1] * matB[3] + matA[2] * matB[6];
	matC[1] = matA[0] * matB[1] + matA[1] * matB[4] + matA[2] * matB[7];
	matC[2] = matA[0] * matB[2] + matA[1] * matB[5] + matA[2] * matB[8];
	matC[3] = matA[3] * matB[0] + matA[4] * matB[3] + matA[5] * matB[6];
	matC[4] = matA[3] * matB[1] + matA[4] * matB[4] + matA[5] * matB[7];
	matC[5] = matA[3] * matB[2] + matA[4] * matB[5] + matA[5] * matB[8];
	matC[6] = matA[6] * matB[0] + matA[7] * matB[3] + matA[8] * matB[6];
	matC[7] = matA[6] * matB[1] + matA[7] * matB[4] + matA[8] * matB[7];
	matC[8] = matA[6] * matB[2] + matA[7] * matB[5] + matA[8] * matB[8];
}

