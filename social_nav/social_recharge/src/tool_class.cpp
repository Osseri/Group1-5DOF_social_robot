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

//Covariance and SVD
int tool_class::SVDwithCov(vector<Point3D> input, Vector3D* eigen_Vector, double *eigen_Value, Point3D& mean_point)
{
	double covMat[9];
	if(makeCovMat(input, mean_point, covMat) == ERROR) return ERROR;
	SVD(covMat, eigen_Vector, eigen_Value);

	//for(int i = 0; i < 9; i++)	printf("Cov[%d] : %lf\n", i, covMat[i]);
	//for(int i = 0; i < 3; i++)	printf("eigen_value[%d] : %lf\n", i, eigen_Value[i]);
	//for(int i = 0; i < 3; i++)	printf("eigen_vec[%d] X : %lf\tY : %lf\tZ : %lf\n", i, eigen_Vector[i].X,  eigen_Vector[i].Y,  eigen_Vector[i].Z);
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

//Fitting Plane
Plane3D tool_class::fit_plane(vector<Point3D> input)
{
	Plane3D plane;
	double eigen_Value[3];
	Vector3D eigen_Vector[3];
	Point3D mean_point;

	memset(&mean_point, 0, sizeof(Point3D));
	memset(eigen_Value, 0, sizeof(double) *3);
	memset(eigen_Vector, 0, sizeof(Vector3D) *3);

	SVDwithCov( input, eigen_Vector, eigen_Value, mean_point);
	eigen_Vector[2] = make_unitvector(eigen_Vector[2]);

	plane.equ_a = eigen_Vector[2].X;
	plane.equ_b = eigen_Vector[2].Y;
	plane.equ_c = eigen_Vector[2].Z;
	plane.equ_d = -(plane.equ_a*mean_point.x + plane.equ_b*mean_point.y + plane.equ_c*mean_point.z);;

	plane.normal_vec = eigen_Vector[2];
	plane.center_position = mean_point;
	plane.points = input;

	//printf("Point Size : %d\n", (int)plane.points.size());
	//printf("mean Point X : %0.3lf\tY : %0.3lf\tZ : %0.3lf\n", mean_point.x, mean_point.y, mean_point.z);
	//printf("Normal_Vector X : %lf\tY : %lf\tZ : %lf\n", plane.normal_vec.X,  plane.normal_vec.Y,  plane.normal_vec.Z);

	return plane;
}

//Fitting Line2D
Line2D tool_class::fit_line2D(vector<Point2D> points) //y = bx + a
{
	Line2D line_2d;
	line_2d.init();

	if(points.size() < 3)
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

	line_2d.distance = distance_BetweenPoint2DAndPoint2D(temp_points[0], temp_points[num_point -1]);
	line_2d.center = temp_points[num_point/2];
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
	line_2d.points = points;

	return line_2d;
}

void tool_class::cal_line_error(vector<Point2D> points, Line2D line, double* output) //[0] aver, [1]min, [2]max
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

void tool_class::multiple_matrix3x3(double* matA, double* matB, double* matC)
{

}

Point2D tool_class::covertRangeToAxis(double range, double angle)
{
	Point2D axis;

	axis.x = range * cos(angle*DEGREE_TO_RADIAN);
	axis.y = range * sin(angle*DEGREE_TO_RADIAN);

	return axis;
}

