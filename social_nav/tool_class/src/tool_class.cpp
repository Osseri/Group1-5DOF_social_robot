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

//Fitting Line2D
Line2D tool_class::fit_line2D(vector<Point2D> points) //y = bx + a
{
	Line2D line_2d;
	line_2d.init();

	if((int)points.size() < 2)
	{
		printf("[ERROR] 'fit_line2D' Not Enough the points!\n");
		return line_2d;
	}

	vector<double> array_y;
	vector<Point2D> input;
	vector<Point2D> temp_points;
	if((int) array_y.size() > 0) array_y.clear();
	if((int) input.size() > 0) input.clear();
	if((int) temp_points.size() > 0) temp_points.clear();

	input = points;
	for(int i = 0; i < (int) points.size(); i++)	array_y.push_back(points[i].y);
	sort(array_y.begin(), array_y.end());

	for(int i = 0; i < (int) array_y.size(); i++)
	{
		for(int j = 0; j < (int) input.size(); i++)
		{
			if(array_y[i] == input[j].y)
			{
				temp_points.push_back(input[j]);
				input.erase(input.begin() + j);
				break;
			}
		}
	}

	int num_point = (int) temp_points.size();

	line_2d.b = (temp_points[0].y - temp_points[num_point -1].y)/(temp_points[0].x - temp_points[num_point -1].x);
	line_2d.a = temp_points[0].y - (temp_points[0].x * line_2d.b);
	line_2d.distance = distance_BetweenPoint2DAndPoint2D(temp_points[0], temp_points[num_point -1]);
	line_2d.center.x = (temp_points[0].x + temp_points[num_point -1].x)/2;
	line_2d.center.y = (temp_points[0].y + temp_points[num_point -1].y)/2;
	line_2d.points = temp_points;
	return line_2d;
}

Line2D tool_class::fit_line2D_LeastSquare(vector<Point2D> points) //y = bx + a
{
	Line2D line_2d;

	line_2d.init();
	if(points.size() < 3)
	{
		printf("[ERROR] 'fit_line2D' Not Enough the points!\n");
		return line_2d;
	}

	int i, num_point;
	double matrix[4];
	double inverse_matrix[4];
	double desteny_num[2];

	memset(matrix, 0, sizeof(double)*4);
	memset(desteny_num, 0, sizeof(double)*2);

	num_point = (int) points.size();

	for(i=0; i< num_point; i++)
	{
		matrix[0] += 1;
		matrix[1] += points[i].x;
		matrix[2] += points[i].x;
		matrix[3] += points[i].x * points[i].x;

		desteny_num[0] += points[i].y;
		desteny_num[1] += points[i].x * points[i].y;
	}

	double temp = 1/((matrix[0] * matrix[3]) - (matrix[1] * matrix[2]));
	inverse_matrix[0] = temp * matrix[3];
	inverse_matrix[1] = -(temp * matrix[1]);
	inverse_matrix[2] = -(temp * matrix[2]);
	inverse_matrix[3] = temp * matrix[0];

	line_2d.a = (inverse_matrix[0] * desteny_num[0]) + (inverse_matrix[1] * desteny_num[1]);
	line_2d.b = (inverse_matrix[2] * desteny_num[0]) + (inverse_matrix[3] * desteny_num[1]);
	line_2d.distance = distance_BetweenPoint2DAndPoint2D(points[0], points[num_point -1]);
	line_2d.center.x = (points[0].x + points[num_point -1].x)/2;
	line_2d.center.y = (points[0].y + points[num_point -1].y)/2;
	line_2d.points = points;

	return line_2d;
}

//2D Line error
void tool_class::cal_2Dline_error(vector<Point2D> points, Line2D line, double* output) //[0] aver, [1]min, [2]max
{
	double distance = 0;
	double min_distance = 1000000;
	double max_distance = -1000000;

	if((int) points.size() > 1)
	{
		for(int i = 0; i < (int) points.size(); i++)
		{
			double a = fabs(line.b * points[i].x - points[i].y + line.a);
			double b = sqrtf(line.b * line.b + 1);
			distance += (a/b);
			if(min_distance > (a/b)) min_distance = (a/b);
			if(max_distance < (a/b)) max_distance = (a/b);
		}

		distance = distance/points.size();
		output[0] = distance;
		output[1] = min_distance;
		output[2] = max_distance;
	}

	else
	{
		output[0] = -1;
		output[1] = -1;
		output[2] = -1;
	}
}

//Fitting Line3D
Line3D tool_class::fit_line3D(vector<Point3D> input)
{
	Line3D line;

	if(input.size() < 3)
	{
		printf("[ERROR] 'fit_line3D' Not Enough the points!\n");
		return line;
	}

	double eigen_Value[3];
	Vector3D eigen_Vector[3];
	Point3D mean_point;

	memset(&mean_point, 0, sizeof(Point3D));
	memset(eigen_Value, 0, sizeof(double) *3);
	memset(eigen_Vector, 0, sizeof(Vector3D) *3);

	SVDwithCov(input, eigen_Vector, eigen_Value, mean_point);

	line.mean_point = mean_point;
	line.direction_vector = make_unitvector(eigen_Vector[0]);
	line.points = input;

	return line;
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
Vector3D tool_class::RotatedVectorAxisX(double Degree, Vector3D vec)
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

Vector3D tool_class::RotatedVectorAxisY(double Degree, Vector3D vec)
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

Vector3D tool_class::RotatedVectorAxisZ(double Degree, Vector3D vec)
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

//Rotation Point2D
Point2D tool_class::RotatedPoint2D(double Degree, Point2D point)
{
	double r_mat[4];
	double angle = Degree * DEGREE_TO_RADIAN;
	eun_u::Point2D reutrn_point;

	r_mat[0] = cos(angle);	r_mat[1] = -sin(angle);
	r_mat[2] = sin(angle);	r_mat[3] = cos(angle);

	reutrn_point.x = point.x * r_mat[0] + point.y * r_mat[1];
	reutrn_point.y = point.x * r_mat[2] + point.y * r_mat[3];

	return reutrn_point;
}

//Rotation Point
Point3D tool_class::RotatedPointAxisX(double Degree, Point3D point)
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

Point3D tool_class::RotatedPointAxisY(double Degree, Point3D point)
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

Point3D tool_class::RotatedPointAxisZ(double Degree, Point3D point)
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

Point3D tool_class::RotatedPointMatrix(double* rmat, Point3D point)
{
	Point3D temp_point;

	temp_point.x = point.x * rmat[0] + point.y * rmat[1] + point.z * rmat[2];
	temp_point.y = point.x * rmat[3] + point.y * rmat[4] + point.z * rmat[5];
	temp_point.z = point.x * rmat[6] + point.y * rmat[7] + point.z * rmat[8];

	return temp_point;
}

//Range to Point
Point2D tool_class::convertRangeToPoint(double range, double angle)
{
	Point2D point;

	point.x = range * cos(angle*DEGREE_TO_RADIAN);
	point.y = range * sin(angle*DEGREE_TO_RADIAN);

	return point;
}

void tool_class::convertRangeToPoints(sensor_msgs::LaserScan input, vector<Point2D>& output)
{
	double angle;
	eun_u::Point2D axis;
	if((int) output.size() > 0) output.clear();

	for(int i = 1; i < (int) input.ranges.size(); i++)
	{
		if( input.ranges[i] > input.range_min && input.ranges[i] < input.range_max)
		{
			angle = (input.angle_min + (input.angle_increment * i)) * RADIAN_TO_DEGREE;
			axis = convertRangeToPoint(input.ranges[i], angle);
			output.push_back(axis);
		}
	}
}

//Translation Using TF
Point2D tool_class::TranslationPointByTF(geometry_msgs::TransformStamped tf_data, Point2D input)
{
	Point2D output;

	geometry_msgs::PointStamped ori;
	geometry_msgs::PointStamped dst;

	ori.point.x = input.x;
	ori.point.y = input.y;
	ori.point.z = 0;
	tf2::doTransform(ori, dst, tf_data);
	output.x = dst.point.x;
	output.y = dst.point.y;

	return output;
}

void tool_class::TranslationPointsByTF(geometry_msgs::TransformStamped tf_data, vector<Point2D> input, vector<Point2D>& output)
{
	Point2D point;
	if((int) output.size() > 0) output.clear();
	for(int i = 0; i < (int) input.size(); i++)
	{
		point = TranslationPointByTF(tf_data, input[i]);
		output.push_back(point);
	}
}


//Quaternion
Point3D tool_class::QuaternionToEuler(double x, double y, double z, double w)
{
	Point3D rotation;
	tf::Quaternion q(x, y, z, w);
	tf::Matrix3x3 m(q);
	m.getRPY(rotation.x, rotation.y, rotation.z);

	rotation.x = rotation.x * RADIAN_TO_DEGREE;
	rotation.y = rotation.y * RADIAN_TO_DEGREE;
	rotation.z = rotation.z * RADIAN_TO_DEGREE;

	return rotation;
}

void tool_class::EulerToQuaternion(Point3D input, double& x, double& y, double& z, double& w)
{
	tf::Quaternion q = tf::createQuaternionFromRPY( input.x, input.y, input.z);
	x = q.getX();
	y = q.getY();
	z = q.getZ();
	w = q.getX();
}

//Matrix Multiplication C = AB
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

//Covariance and SVD
int tool_class::SVDwithCov(vector<Point3D> input, Vector3D* eigen_Vector, double *eigen_Value, Point3D& mean_point)
{
	double covMat[9];
	if(makeCovMat(input, mean_point, covMat) == ERROR) return ERROR;
	SVD(covMat, eigen_Vector, eigen_Value);

	//for(int i = 0; i < 9; i++)	printf("Cov[%d] : %0.3lf\n", i, covMat[i]);
	//for(int i = 0; i < 3; i++)	printf("eigen_value[%d] : %0.3lf\n", i, eigen_Value[i]);
	//for(int i = 0; i < 3; i++)	printf("eigen_vec[%d] X : %0.3lf\tY : %0.3lf\tZ : %0.3lf\n", i, eigen_Vector[i].X,  eigen_Vector[i].Y,  eigen_Vector[i].Z);

	return NOT_ERROR;
}

int tool_class::makeCovMat(vector<Point3D> input, Point3D &mean, double* out_cov)
{
	int i;
	memset(out_cov, 0, sizeof(double) * 9);
	Point3D* submean = new Point3D[input.size()];
	if(submean == NULL) return ERROR;

	mean = cal_meanPoint3D(input);

	for(i=0; i<(int)input.size(); i++)
	{
		submean[i].x = input[i].x - mean.x;
		submean[i].y = input[i].y - mean.y;
		submean[i].z = input[i].z - mean.z;
	}

	for(i=0; i<(int)input.size(); i++)
	{
		out_cov[0] = out_cov[0] + submean[i].x * submean[i].x;
	}

	for(i=0; i<(int)input.size(); i++)
	{
		out_cov[1] = out_cov[1] + submean[i].x * submean[i].y;
	}

	for(i=0; i<(int)input.size(); i++)
	{
		out_cov[2] = out_cov[2] + submean[i].x * submean[i].z;
	}

	for(i=0; i<(int)input.size(); i++)
	{
		out_cov[3] = out_cov[3] + submean[i].y * submean[i].x;
	}

	for(i=0; i<(int)input.size(); i++)
	{
		out_cov[4] = out_cov[4] + submean[i].y * submean[i].y;
	}

	for(i=0; i<(int)input.size(); i++)
	{
		out_cov[5] = out_cov[5] + submean[i].y * submean[i].z;
	}

	for(i=0; i<(int)input.size(); i++)
	{
		out_cov[6] = out_cov[6] + submean[i].z * submean[i].x;
	}

	for(i=0; i<(int)input.size(); i++)
	{
		out_cov[7] = out_cov[7] + submean[i].z * submean[i].y;
	}

	for(i=0; i<(int)input.size(); i++)
	{
		out_cov[8] = out_cov[8] + submean[i].z * submean[i].z;
	}

	if(submean) { delete [] submean; submean = NULL; }
	return	NOT_ERROR;
}

void tool_class::SVD(double* covMat, Vector3D* eigen_Vector, double* eigen_Value)
{
	MatrixXf mat = MatrixXf::Random(3,3);
	mat(0,0) = covMat[0];
	mat(0,1) = covMat[1];
	mat(0,2) = covMat[2];
	mat(1,0) = covMat[3];
	mat(1,1) = covMat[4];
	mat(1,2) = covMat[5];
	mat(2,0) = covMat[6];
	mat(2,1) = covMat[7];
	mat(2,2) = covMat[8];

	JacobiSVD<MatrixXf> svd(mat, ComputeThinU | ComputeThinV);
	//cout << "Its left singular vectors are the columns of the thin U matrix:" << endl << svd.matrixU() << endl;
	//cout << "Its right singular vectors are the columns of the thin V matrix:" << endl << svd.matrixV() << endl;

	MatrixXf value = svd.singularValues();
	eigen_Value[0] = value(0);
	eigen_Value[1] = value(1);
	eigen_Value[2] = value(2);

	MatrixXf vec = svd.matrixV();
	eigen_Vector[0].X = vec(0,0);
	eigen_Vector[0].Y = vec(1,0);
	eigen_Vector[0].Z = vec(2,0);
	eigen_Vector[1].X = vec(0,1);
	eigen_Vector[1].Y = vec(1,1);
	eigen_Vector[1].Z = vec(2,1);
	eigen_Vector[2].X = vec(0,2);
	eigen_Vector[2].Y = vec(1,2);
	eigen_Vector[2].Z = vec(2,2);

	eigen_Vector[0] = make_unitvector(eigen_Vector[0]);
	eigen_Vector[1] = make_unitvector(eigen_Vector[1]);
	eigen_Vector[2] = make_unitvector(eigen_Vector[2]);
}

//Cal Vector Angle YZ 변환기준 -> 역변환 ZY X축 맞추기
void tool_class::cal_vector_angle(Vector3D input, double& angle_y, double& angle_z)
{
	double angle;
	Vector3D dir_vec;
	Vector3D temp_vec;

	//Cal angle_z
	dir_vec = temp_vec = input;
	temp_vec.Z = 0;
	angle = angle_BetweenAxisXandAvector(temp_vec);
	if(temp_vec.Y > 0) angle_z = fabs(angle);
	else angle_z = -fabs(angle);

	//Cal angle_y
	temp_vec = RotatedVectorAxisZ(-angle_z, dir_vec);
	angle = angle_BetweenAxisXandAvector(temp_vec);
	if(temp_vec.Z < 0) angle_y = fabs(angle);
	else angle_y = -fabs(angle);

	/*
	temp_vec = RoatedVectorAxisZ(-angle_z, dir_vec);
	temp_vec = RoatedVectorAxisY(-angle_y, temp_vec);
	printf("Cal_Vecor_Angle X : %0.3lf\tY : %0.3lf\tZ : %0.3lf\n", temp_vec.X,  temp_vec.Y,  temp_vec.Z);
	*/
}

void tool_class::cal_pose_angle(Vector3D* input, double& angle_x, double& angle_y, double& angle_z) //
{
	double angle;
	Vector3D x_vec;
	Vector3D y_vec;
	Vector3D temp_vec;

	//Cal angle_z
	x_vec = temp_vec = input[0];
	temp_vec.Z = 0;
	angle = angle_BetweenAxisXandAvector(temp_vec);
	if(temp_vec.Y > 0) angle_z = fabs(angle);
	else angle_z = -fabs(angle);

	//Cal angle_y
	temp_vec = RotatedVectorAxisZ(-angle_z, x_vec);
	angle = angle_BetweenAxisXandAvector(temp_vec);
	if(temp_vec.Z < 0) angle_y = fabs(angle);
	else angle_y = -fabs(angle);

	//Cal angle_x
	y_vec = input[1];
	temp_vec = RotatedVectorAxisZ(-angle_z, y_vec);
	temp_vec = RotatedVectorAxisY(-angle_y, temp_vec);
	angle = angle_BetweenAxisYandAvector(temp_vec);
	if(temp_vec.Z > 0) angle_x = fabs(angle);
	else angle_x = -fabs(angle);

	/*
	Vector3D z_vec = input[2];
	temp_vec = RoatedVectorAxisZ(-angle_z, x_vec);
	temp_vec = RoatedVectorAxisY(-angle_y, temp_vec);
	temp_vec = RoatedVectorAxisX(-angle_x, temp_vec);
	printf("Cal_Pose Xvec X : %0.3lf\tY : %0.3lf\tZ : %0.3lf\n", temp_vec.X,  temp_vec.Y,  temp_vec.Z);
	temp_vec = RoatedVectorAxisZ(-angle_z, y_vec);
	temp_vec = RoatedVectorAxisY(-angle_y, temp_vec);
	temp_vec = RoatedVectorAxisX(-angle_x, temp_vec);
	printf("Cal_Pose_Yvec X : %0.3lf\tY : %0.3lf\tZ : %0.3lf\n", temp_vec.X,  temp_vec.Y,  temp_vec.Z);
	temp_vec = RoatedVectorAxisZ(-angle_z, z_vec);
	temp_vec = RoatedVectorAxisY(-angle_y, temp_vec);
	temp_vec = RoatedVectorAxisX(-angle_x, temp_vec);
	printf("Cal_Pose_Zvec X : %0.3lf\tY : %0.3lf\tZ : %0.3lf\n", temp_vec.X,  temp_vec.Y,  temp_vec.Z);
	*/
}

void tool_class::cal_rotation_mat_XYZ(double angle_x, double angle_y, double angle_z, double* output)
{
	double temp[9];
	double mat_x[9], mat_y[9], mat_z[9];
	mat_x[0] = 1;	mat_x[1] = 0;				mat_x[2] = 0;
	mat_x[3] = 0;	mat_x[4] = cos(angle_x*DEGREE_TO_RADIAN);	mat_x[5] = -sin(angle_x*DEGREE_TO_RADIAN);
	mat_x[6] = 0;	mat_x[7] = sin(angle_x*DEGREE_TO_RADIAN);	mat_x[8] = cos(angle_x*DEGREE_TO_RADIAN);

	mat_y[0] = cos(angle_y*DEGREE_TO_RADIAN);		mat_y[1] = 0;	mat_y[2] = sin(angle_y*DEGREE_TO_RADIAN);
	mat_y[3] = 0;										mat_y[4] = 1;	mat_y[5] = 0;
	mat_y[6] = -sin(angle_y*DEGREE_TO_RADIAN);	mat_y[7] = 0;	mat_y[8] = cos(angle_y*DEGREE_TO_RADIAN);

	mat_z[0] = cos(angle_z*DEGREE_TO_RADIAN);	mat_z[1] = -sin(angle_z*DEGREE_TO_RADIAN);	mat_z[2] = 0;
	mat_z[3] = sin(angle_z*DEGREE_TO_RADIAN);	mat_z[4] = cos(angle_z*DEGREE_TO_RADIAN);		mat_z[5] = 0;
	mat_z[6] = 0;				mat_z[7] = 0;				mat_z[8] = 1;

	multiple_matrix3x3(mat_x, mat_y, temp);
	multiple_matrix3x3(temp, mat_z, output);
}

void tool_class::cal_rotation_mat_ZYX(double angle_x, double angle_y, double angle_z, double* output)
{
	double temp[9];
	double mat_x[9], mat_y[9], mat_z[9];
	mat_x[0] = 1;	mat_x[1] = 0;				mat_x[2] = 0;
	mat_x[3] = 0;	mat_x[4] = cos(angle_x*DEGREE_TO_RADIAN);	mat_x[5] = -sin(angle_x*DEGREE_TO_RADIAN);
	mat_x[6] = 0;	mat_x[7] = sin(angle_x*DEGREE_TO_RADIAN);	mat_x[8] = cos(angle_x*DEGREE_TO_RADIAN);

	mat_y[0] = cos(angle_y*DEGREE_TO_RADIAN);		mat_y[1] = 0;	mat_y[2] = sin(angle_y*DEGREE_TO_RADIAN);
	mat_y[3] = 0;										mat_y[4] = 1;	mat_y[5] = 0;
	mat_y[6] = -sin(angle_y*DEGREE_TO_RADIAN);	mat_y[7] = 0;	mat_y[8] = cos(angle_y*DEGREE_TO_RADIAN);

	mat_z[0] = cos(angle_z*DEGREE_TO_RADIAN);	mat_z[1] = -sin(angle_z*DEGREE_TO_RADIAN);	mat_z[2] = 0;
	mat_z[3] = sin(angle_z*DEGREE_TO_RADIAN);	mat_z[4] = cos(angle_z*DEGREE_TO_RADIAN);		mat_z[5] = 0;
	mat_z[6] = 0;				mat_z[7] = 0;				mat_z[8] = 1;

	multiple_matrix3x3(mat_z, mat_y, temp);
	multiple_matrix3x3(temp, mat_x, output);
}

//Plane Modeling
eun_u::Plane3D tool_class::cal_plane_data(eun_u::Object3D input)
{
	eun_u::Plane3D return_plane;

	if((int) input.points.size() > 3)
	{
		//Plane mean Center and Pose
		double eigen_value[3];
		eun_u::Vector3D dir_vec[3], eigen_vector[3];
		SVDwithCov(input.points, eigen_vector, eigen_value, return_plane.mean_center);
		return_plane.vec_x = dir_vec[0] = eigen_vector[2];	//XYZ 벡터 정렬
		return_plane.vec_y = dir_vec[1] = eigen_vector[0];
		return_plane.vec_z = dir_vec[2] = eigen_vector[1];
		cal_pose_angle(dir_vec, return_plane.angle_x, return_plane.angle_y, return_plane.angle_z);

		//Plane median Center
		return_plane.median_center = cal_medianPoint3D(input.points);

		//Plane Equation
		return_plane.equ_a = dir_vec[2].X;
		return_plane.equ_b = dir_vec[2].Y;
		return_plane.equ_c = dir_vec[2].Z;
		return_plane.equ_d = -(return_plane.equ_a*return_plane.median_center.x + return_plane.equ_b*return_plane.median_center.y + return_plane.equ_c*return_plane.median_center.z);
		return_plane.points = input.points;

		//Plane Modeling
		plane_modeling(return_plane);
	}

	else ROS_INFO("Not Enough Points!(tool_class|cal_plane_data)");

	return return_plane;
}

void tool_class::plane_modeling(eun_u::Plane3D& plane)
{
	Bounds3D range;
	vector<Point3D> points;
	vector<Point3D> sorting;

	int sort_size;
	double r_mat[9], inverse_mat[9];

	//cal rotation matrix and rotation plane points
	cal_rotation_mat_XYZ(-plane.angle_x, -plane.angle_y, -plane.angle_z, r_mat);
	cal_rotation_mat_ZYX(plane.angle_x, plane.angle_y, plane.angle_z, inverse_mat);
	if((int) points.size() > 0) points.clear();

	//cal Plane Size & Size Center
	for(int i = 0; i < (int) plane.points.size(); i++)	points.push_back(RotatedPointMatrix(r_mat, plane.points[i]));
	range = plane_range(points);
	if(plane.width < 100 && plane.height < 100)
	{
		plane.width = range.max_y - range.min_y;
		plane.height = range.max_z - range.min_z;
		plane.size_center.x = (range.max_x + range.min_x)/2;
		plane.size_center.y = (range.max_y + range.min_y)/2;
		plane.size_center.z = (range.max_z + range.min_z)/2;
		plane.size_center = RotatedPointMatrix(inverse_mat, plane.size_center);
		//printf("Dis X : %0.3lf\tY : %0.3lf\tZ : %0.3lf\n", range.max_x - range.min_x, plane.width, plane.height);
		//printf("Size Center X : %0.3lf\tY : %0.3lf\tZ : %0.3lf\n", plane.size_center.x, plane.size_center.y, plane.size_center.z);
		//printf("range max_z : %0.3lf\tmin_z : %0.3lf", range.max_z, range.min_z);
	}

	else
	{
		plane.width = 0;
		plane.height = 0;
		plane.size_center.init();
		ROS_INFO("Invalid value plane width or plane height!(tool_class|plane_modeling)");
	}

	//Plane Point Sorting
	if(plane.width * plane.height > MIN_PLANE_SIZE)
	{
		//Cal octagon point
		point_sortingYZ(points, range, sort_size, sorting);
		cal_octagon_point(sort_size, sorting, plane.octagon_poins);
		for(int i = 0; i < 8; i++) plane.octagon_poins[i] = RotatedPointMatrix(inverse_mat, plane.octagon_poins[i]);
		//for(int i = 0; i < (int) sorting.size(); i++)
			//printf("[%d] X : %0.3lf\tY : %0.3lf\tZ : %0.3lf\n", i, sorting[i].x, sorting[i].y, sorting[i].z);
	}

	else ROS_INFO("Plane is so Small and Not enough Size Octagon Modeling!(tool_class|plane_modeling)");

	if((int) points.size() > 0 ) points.clear();
	if((int) sorting.size() > 0 ) sorting.clear();
}

Bounds3D tool_class::plane_range(vector <Point3D> input)
{
	Bounds3D output;

	if((int) input.size() > 3)
	{
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

		output.min_x = array_x[0];
		output.max_x = array_x[(int)array_x.size() - 1];
		output.min_y = array_y[0];
		output.max_y = array_y[(int)array_y.size() - 1];
		output.min_z = array_z[0];
		output.max_z = array_z[(int)array_z.size() - 1];

		if((int) array_x.size() > 0) array_x.clear();
		if((int) array_y.size() > 0) array_y.clear();
		if((int) array_z.size() > 0) array_z.clear();
	}

	else
	{
		ROS_INFO("Not Enough Points!(tool_class|plane_range)");
		output.init();
	}
	return output;
}

void tool_class::point_sortingYZ(vector<Point3D> input, Bounds3D range, int& size, vector<Point3D>& output_points)
{
	int p_width = (int) ((range.max_y - range.min_y)/SORTING_SIZE) + 5;
	int p_height = (int) ((range.max_z - range.min_z)/SORTING_SIZE) + 5;
	if(p_width > p_height) size = p_width;
	else size = p_height;
	Point3D center;
	PointUV center_index;

	center.x = 0;
	center.y = (range.max_y + range.min_y)/2;
	center.z = (range.max_z + range.min_z)/2;
	center_index.u = size/2;
	center_index.v = size/2;

	int w_index;
	int h_index;
	int p_index;
	if((int) output_points.size() > 0) output_points.clear();

	if(size > 0 && size < 500)
	{
		point_box *temp_box = new point_box [size * size];
		for(int i = 0; i < input.size(); i++)
		{
			w_index = ((int)((input[i].y - center.y)/SORTING_SIZE)) + center_index.u;
			h_index = ((int)((input[i].z - center.z)/SORTING_SIZE)) + center_index.v;
			//printf("[%d] W : %d\tH : %d\tY : %0.3lf\tZ : %0.3lf\n", i, w_index, h_index, input->points[i].y, input->points[i].z);
			if(w_index > -1 && w_index < size && h_index > -1 && h_index < size)
			{
				p_index = h_index * size + w_index;
				temp_box[p_index].points.push_back(input[i]);
			}
		}

		Point3D temp_point;
		for(int i = 0; i < size * size; i++)
		{
			if((int) temp_box[i].points.size() > 0)
			{
				temp_point = cal_medianPoint3D(temp_box[i].points);
			}

			else
			{
				temp_point.init();
			}
			output_points.push_back(temp_point);
		}

		delete[] temp_box;
	}

	else
	{
		size = 0;
		if((int) output_points.size() > 0) output_points.clear();
		ROS_INFO("Invalid value plane width or plane height!(tool_class|point_sortingYZ)");
	}
}

void tool_class::cal_octagon_point(int size, vector<Point3D> input, Point3D* output)
{
	//test
	double a = 5.0;
	double b = 2.0;

	int p_index;
	int half_size = size/2;
	//memset(output, 0, sizeof(Point3D) * 8);
	if(size > 0 && size < 500 && (int) input.size())
	{
		p_index = half_size * size + half_size;
		//printf("Size : %d\n", size);
		//printf("Center X : %0.3lf\tY : %0.3lf\tZ : %0.3lf\n", input[p_index].x, input[p_index].y, input[p_index].z);
		//팔각형의 0 Point
		for(int i = 0; i < half_size; i++)
		{
			p_index = i * size + half_size;
			if(input[p_index].x != 0 && input[p_index].y != 0 && input[p_index].z != 0)
			{
				output[0] = input[p_index];
				//printf("P[0] X : %0.3lf\tY : %0.3lf\tZ : %0.3lf\n", output[0].x, output[0].y, output[0].z);
				break;
			}
		}

		//팔각형의 1 Point
		for(int i = 0; i < half_size; i++)
		{
			p_index = i * size + i;
			if(input[p_index].x != 0 && input[p_index].y != 0 && input[p_index].z != 0)
			{
				output[1] = input[p_index];
				//printf("P[1] %d\tX : %0.3lf\tY : %0.3lf\tZ : %0.3lf\n", i, output[1].x, output[1].y, output[1].z);
				break;
			}
		}

		//팔각형의 2 Point
		for(int i = 0; i < half_size; i++)
		{
			p_index = half_size * size + i;
			if(input[p_index].x != 0 && input[p_index].y != 0 && input[p_index].z != 0)
			{
				output[2] = input[p_index];
				break;
			}
		}

		//팔각형의 3 Point (역방향)
		for(int i = size; i > half_size; i--)
		{
			p_index = (i-1) * size + (size - i);
			if(input[p_index].x != 0 && input[p_index].y != 0 && input[p_index].z != 0)
			{
				output[3] = input[p_index];
				break;
			}
		}

		//팔각형의 4 Point	(역방향)
		for(int i = size; i > half_size; i--)
		{
			p_index = (i-1) * size + half_size;
			if(input[p_index].x != 0 && input[p_index].y != 0 && input[p_index].z != 0)
			{
				output[4] = input[p_index];
				break;
			}
		}

		//팔각형의 5 Point (역방향)
		for(int i = size; i > half_size; i--)
		{
			p_index = (i-1) * size + (i - 1);
			if(input[p_index].x != 0 && input[p_index].y != 0 && input[p_index].z != 0)
			{
				output[5] = input[p_index];
				break;
			}
		}

		//팔각형의 6 Point
		for(int i = 0; i < half_size; i++)
		{
			p_index = half_size * size + (size - i - 1);
			if(input[p_index].x != 0 && input[p_index].y != 0 && input[p_index].z != 0)
			{
				output[6] = input[p_index];
				break;
			}
		}

		//팔각형의 7 Point
		for(int i = 0; i < half_size; i++)
		{
			p_index = i * size + (size - i - 1);
			if(input[p_index].x != 0 && input[p_index].y != 0 && input[p_index].z != 0)
			{
				output[7] = input[p_index];
				break;
			}
		}
	}

	else
	{
		ROS_INFO("Invalid value plane width or plane height!(tool_class|cal_octagon_point)");
	}
}


//Object Modeling
eun_u::Object3D tool_class::cal_object_data(eun_u::Object3D input)
{
	eun_u::Object3D return_object;

	if((int) input.points.size() > 3)
	{
		//Plane mean Center and Pose
		eun_u::Vector3D dir_vec[3];
		SVDwithCov(input.points, return_object.eigen_vector, return_object.eigen_value, return_object.mean_center);
		dir_vec[0] = return_object.eigen_vector[2];	//XYZ 벡터 정렬
		dir_vec[1] = return_object.eigen_vector[0];
		dir_vec[2] = return_object.eigen_vector[1];
		cal_pose_angle(dir_vec, return_object.angle_x, return_object.angle_y, return_object.angle_z);

		//Plane median Center
		return_object.median_center = cal_medianPoint3D(input.points);
		return_object.points = input.points;
		object_modeling(return_object);
	}

	else ROS_INFO("Not Enough Points!(tool_class|cal_Object_data)");

	return return_object;
}

void tool_class::object_modeling(eun_u::Object3D& input)
{
	Bounds3D range;
	vector<Point3D> points;

	double r_mat[9], inverse_mat[9];

	//cal rotation matrix and rotation plane points
	cal_rotation_mat_XYZ(-input.angle_x, -input.angle_y, -input.angle_z, r_mat);
	cal_rotation_mat_ZYX(input.angle_x, input.angle_y, input.angle_z, inverse_mat);
	if((int) points.size() > 0) points.clear();

	//cal Plane Size & Size Center
	for(int i = 0; i < (int) input.points.size(); i++)	points.push_back(RotatedPointMatrix(r_mat, input.points[i]));
	range = plane_range(points);
	if(input.width < 100 && input.height < 100)
	{
		input.width = range.max_y - range.min_y;
		input.height = range.max_z - range.min_z;
		input.size_center.x = (range.max_x + range.min_x)/2;
		input.size_center.y = (range.max_y + range.min_y)/2;
		input.size_center.z = (range.max_z + range.min_z)/2;
		input.size_center = RotatedPointMatrix(inverse_mat, input.size_center);
		//printf("Dis X : %0.3lf\tY : %0.3lf\tZ : %0.3lf\n", range.max_x - range.min_x, plane.width, plane.height);
		//printf("Size Center X : %0.3lf\tY : %0.3lf\tZ : %0.3lf\n", plane.size_center.x, plane.size_center.y, plane.size_center.z);
		//printf("range max_z : %0.3lf\tmin_z : %0.3lf", range.max_z, range.min_z);
	}

	else
	{
		input.width = 0;
		input.height = 0;
		input.size_center.init();
		ROS_INFO("Invalid value plane width or plane height!(tool_class|object_modeling)");
	}

	if((int) points.size() > 0 ) points.clear();
}

