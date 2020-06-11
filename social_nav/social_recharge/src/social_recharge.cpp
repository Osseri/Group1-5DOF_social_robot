
#include "social_recharge.h"
#include <math.h>

// init
social_recharge::social_recharge()
{
  //기본 값 init
  m_charge = false;
  m_pose.init();
  m_laser.clear();

  m_nodeHandle.param("social_recharge_node/tarket_distance", m_tarket_dis, 0.3);
  m_nodeHandle.param("social_recharge_node/trans_threshold", m_trans_threshold, 0.03);
  m_nodeHandle.param("social_recharge_node/angle_threshold", m_angle_threshold, 10.0);
  m_nodeHandle.param("social_recharge_node/min_x", m_laser_area.min_x, 0.0);
  m_nodeHandle.param("social_recharge_node/max_x", m_laser_area.max_x, 2.0);
  m_nodeHandle.param("social_recharge_node/min_y", m_laser_area.min_y, -2.0);
  m_nodeHandle.param("social_recharge_node/max_y", m_laser_area.max_y, 2.0);
  m_nodeHandle.param("social_recharge_node/min_z", m_laser_area.min_z, 0.0);
  m_nodeHandle.param("social_recharge_node/leave_distance", m_leave_dis, 0.7);

  //printf("trans_threshold : %0.3lf\tangle_threshold : %0.3lf\n", m_trans_threshold, m_angle_threshold);
  //printf("Min Y : %0.3lf\tma Y : %0.3lf\n", m_laser_area.min_y, m_laser_area.max_y);

  init_node();
}

social_recharge::~social_recharge()
{

}

void social_recharge::init_node()
{
  //Navigation
  m_req_docking = m_nodeHandle.advertiseService("/social_recharge/dock_on_station",  &social_recharge::docking_ServiceCallback, this);    // station docking
  m_req_leaving = m_nodeHandle.advertiseService("/social_recharge/leave_station",  &social_recharge::leaving_ServiceCallback, this);      // station leaving
  m_sub_charge = m_nodeHandle.subscribe("/charger_dock_switch", 1, &social_recharge::charge_SubscribeCallBack, this);                               // is docking check
  m_sub_laser_scan = m_nodeHandle.subscribe("/scan", 1, &social_recharge::laser_SubscribeCallBack, this);                               //
  m_sub_odom = m_nodeHandle.subscribe("/odom", 1, &social_recharge::odom_SubscribeCallBack, this);

  m_pub_marker = m_nodeHandle.advertise<visualization_msgs::Marker>("/social_recharge/marker", 10);
  m_pub_wheel = m_nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  m_pub_dock = m_nodeHandle.advertise<std_msgs::Bool>("/social_recharge/dock_state", 10);

}

//Docking Start
bool social_recharge::docking_ServiceCallback(robocare_msgs::DockOnStation::Request &request, robocare_msgs::DockOnStation::Response &response)
{
  printf("Docking Start!\n");
  //  if(m_charge == false)
  //  {
  docking_msg.data = true;
  laser_moving();		//Laser인식을 기본으로 충전기로 이동
  m_pub_dock.publish(docking_msg);

  ros::Duration(0.5).sleep();
  //        psd_moving();	//Ir 값을 바탕으로 최종 도킹

  response.message ="Complete";
  response.success = true;

  //  }

  //  else
  //  {
  //    response.message ="Already docked";
  //    response.success = true;
  //  }

  // [Question] Docking complete message??

  printf("Docking End!\n");
  return true;
}

bool social_recharge::leaving_ServiceCallback(robocare_msgs::LeaveStation::Request &request, robocare_msgs::LeaveStation::Response &response)
{

  // 출발하면 docking false msg 날림
  // docking_msg.data = false;
  // m_pub_dock.publish(docking_msg);
  // ros::Duration(1).sleep();
  
  wheelmoveX(m_leave_dis, 0.1);		//200mm/s 속도로 설정거리만큼 이동.
  response.message ="Complete";
  response.success = true;

  //  if(m_charge == true)	//충전 중이거나 바로뒤에 벽이 있을 경우
  //  {
  //    // 출발하면 docking false msg 날림
  //    docking_msg.data = false;
  //    m_pub_dock.publish(docking_msg);

  //    wheelmoveX(m_leave_dis, 0.1);		//200mm/s 속도로 설정거리만큼 이동.
  //    response.message ="Complete";
  //    response.success = true;
  //  }

  //  else
  //  {
  //    response.message ="Not recharging state";		//충전상태가 아님을 고지.
  //    response.success = true;
  //  }

  return true;
}

void social_recharge::charge_SubscribeCallBack(const std_msgs::Bool& input)
{
  //printf("data [0] : %d\t[1] : %d\[2] : %d\n", input->data[0], input->data[1], input->data[2]);
  if(input.data == true) m_charge = true;
  else m_charge = false;
  ros::Duration(0.01).sleep();
}



void social_recharge::laser_SubscribeCallBack(const sensor_msgs::LaserScan::ConstPtr& input)      //
{
  double angle;
  double range;
  eun_u::Point2D axis;
  vector<double> axis_y;
  vector<eun_u::Point2D> points;
  vector<eun_u::Point2D> laser;
  if((int) axis_y.size() > 0) axis_y.clear();
  if((int) points.size() > 0) points.clear();
  if((int) laser.size() > 0) laser.clear();

  for(int i = 1; i < input->ranges.size(); i++)
  {
    if( input->ranges[i] > input->range_min && input->ranges[i] < input->range_max)             //레이저값이 유효한 거리인지 확인
    {
      angle = (input->angle_min + (input->angle_increment * i)) * RADIAN_TO_DEGREE + 180; //Range값의 각도값 계산. 180
      axis = mat_cal.covertRangeToAxis(input->ranges[i], angle);                          //Range와 각도값을 이용 Point값으로 변환

      if(axis.x > m_laser_area.min_x && axis.x < m_laser_area.max_x &&
         axis.y > m_laser_area.min_y && axis.y < m_laser_area.max_y)         //유효영역내의 포인트 필터링 후 저장
      {
        points.push_back(axis);
        axis_y.push_back(axis.y);
      }
    }
  }

  if((int)points.size() > 0)	//포인트값을 Y값에 따라서 정렬
  {
    sort(axis_y.begin(), axis_y.end());
    for(int i = 0; i < (int) axis_y.size(); i++)
    {
      for(int j = 0; j < (int) points.size(); j++)
      {
        if(axis_y[i] == points[j].y)
        {
          laser.push_back(points[j]);
          points.erase(points.begin() + j);
          break;
        }
      }
    }
  }

  if((int) m_laser.size() > 0) m_laser.clear();
  m_laser = laser;

  if((int) axis_y.size() > 0) axis_y.clear();
  if((int) points.size() > 0) points.clear();
  if((int) laser.size() > 0) laser.clear();
  ros::Duration(0.01).sleep();
}

void social_recharge::odom_SubscribeCallBack(const nav_msgs::Odometry::ConstPtr& input)
{
  eun_u::Point3D rotation;
  rotation = mat_cal.QuaternionToEuler(input->pose.pose.orientation.x, input->pose.pose.orientation.y, input->pose.pose.orientation.z, input->pose.pose.orientation.w);
  m_pose.x = input->pose.pose.position.x;
  m_pose.y = input->pose.pose.position.y;
  m_pose.z = rotation.z;
}


void social_recharge::laser_moving()
{
  //First Moving
  ros::Rate r(9);
  vector<eun_u::Point2D> laser; // 복사 할 laser vector
  eun_u::Point3D target_position;


  while(ros::ok())
  {
    if((int) laser.size() > 0) laser.clear();
    laser = m_laser;		//Laser 센서값 저장

    if((int) laser.size() > 5)
    {
      target_position = find_target(laser);	//충전기 위치 인식

      //Moving
      if(target_position.x == 0 && target_position.y == 0)	//충전기 위치 불인식시 정지
      {
        target_position.init();
        wheel_command_laser_detail(target_position);
      }

      else if(target_position.x > 0.5)	//충전기 위치가 500mm 이상 떨어졌을 대 러프이동
      {
        wheel_command_laser_rough(target_position);
      }

      else if(target_position.x > 0.2 || fabs(target_position.z) > m_angle_threshold)	//충전기 위치가 400mm 이상 떨어졌을 대 정밀이동
      {
        wheel_command_laser_detail(target_position);
      }

      else if (target_position.x < 0.2 ) //&& m_charge == true)
      {
        ROS_INFO("SIGNAL IN !!!!\n");
        target_position.init();
        wheel_command_laser_detail(target_position);
        break;

      }

      else if (target_position.x < 0.182)		//충전기 위치가 4000mm이하 각도도 문턱치 이하일 때 laser를 이용한 이동 종료
      {
        target_position.init();
        wheel_command_laser_detail(target_position);
        break;
      }
    }

    else
    {
      target_position.init();
      wheel_command_laser_detail(target_position);
    }

    if((int) laser.size() > 0) laser.clear();
    r.sleep();
  }
  if((int) laser.size() > 0) laser.clear();
}

eun_u::Point3D social_recharge::find_target(vector<eun_u::Point2D> laser)
{
  eun_u::Line2D tarket_line;
  eun_u::Point3D target_position;
  vector<eun_u::Line2D> line;
  vector<eun_u::Point2D> points;

  //Object Detector
  points = laser;		//센서값 빽업
  detect_line(points, line);		//입력 laser값에서 Line 추출

  if((int)line.size() > 0)	tarket_line = select_line(line);	//추출된 라인중 충전기 라인 찾기
  else tarket_line.init();	//추출된 라인이 없을 시 초기화
  //printf("Num Points : %d\n", (int) points.size());
  //printf("Num Line : %d\n", (int) line.size());

  if((int) tarket_line.points.size() > 0)
  {
    target_position = cal_tarket_position(tarket_line);		//충전기 라인에서 충전기 위치 추출
  }

  else
  {
    target_position.init();
    printf("No Target\n");
  }

  line_display(tarket_line, line);	//인식된 충전기 및 인식된 Line Display

  if((int) line.size() > 0) line.clear();
  if((int) points.size() > 0) points.clear();
  return target_position;
}


void social_recharge::detect_line(vector<eun_u::Point2D> input, vector<eun_u::Line2D>& out_put)
{
  if((int) input.size() > 3)
  {
    double distance;
    eun_u::Point2D temp_point;
    vector<eun_u::Point2D> points;
    vector<eun_u::Point2D> object_points;
    vector<eun_u::Line2D> temp_lines;
    if((int) object_points.size() > 0) object_points.clear();
    if((int) temp_lines.size() > 0) temp_lines.clear();
    points = input;

    //Detect Object(리즌 글로윙을 통한 Line Detection)
    while((int) points.size() > 5)
    {
      int count = 0;
      for(int i = 0; i < (int) points.size() - 1; i++)
      {
        if(count == 0)
        {
          distance = mat_cal.distance_BetweenPoint2DAndPoint2D(points[i], points[i+1]);
          if(distance < 0.07)
          {
            object_points.push_back(points[i]);
            points.erase(points.begin() + i);
            i--;
          }

          else
          {
            temp_point = points[i];
            object_points.push_back(points[i]);
            points.erase(points.begin() + i);
            i--;
            count++;
          }
        }

        else
        {
          distance = mat_cal.distance_BetweenPoint2DAndPoint2D(temp_point, points[i+1]);
          if(distance < 0.05)	count = 0;
          else count++;
        }
      }

      if((int) object_points.size() > 5)
      {
        eun_u::Line2D temp_line = mat_cal.fit_line2D(object_points);
        if(temp_line.distance > 0.1 )temp_lines.push_back(temp_line);
      }

      else
      {
        printf("Low Point!\n");
      }

      if((int) object_points.size() > 0) object_points.clear();

      //printf("Num Object : %d\tNum Point : %d\n", (int) temp_objects.size(), (int) points.size());
    }

    if((int) out_put.size() > 0) out_put.clear();

    //Object Sorting
    out_put = temp_lines;

    if((int) points.size() > 0) points.clear();
    if((int) object_points.size() > 0) object_points.clear();
    if((int) temp_lines.size() > 0) temp_lines.clear();
  }

  else
  {
    if((int) out_put.size() > 0) out_put.clear();
  }
}

eun_u::Line2D social_recharge::select_line(vector<eun_u::Line2D>& input)
{
  eun_u::Line2D return_line;

  int line_index = -100;
  double distance, bet_dis, min_distance = 100000;

  //입력된 라인에서 충전기 길이와 로봇과 가장 가까이 있는 물체로 충전기 찾기.
  for(int i = 0; i < (int) input.size(); i++)
  {
    input[i].points.size();
    distance = input[i].distance;
    //printf("[%d] : %0.3lf\n", i, distance);
    if(m_tarket_dis + 0.15 > distance && distance > m_tarket_dis - 0.15 && distance > 0.1)
    {
      distance = mat_cal.distance_BetweenZeroPointAndPoint2D(input[i].center);
      if(distance < min_distance)
      {
        line_index = i;
        min_distance = distance;
      }
    }
  }

  //입력된 Line 중 충전기 제거
  if(line_index != -100)
  {
    return_line = input[line_index];
    input.erase(input.begin() + line_index);
  }

  //로봇과 충전기 사이에 장애물이 있는 지 확인 장애물 확인시 충전기 라인을 초기화
  for(int i = 0; i < (int) input.size(); i++)
  {
    distance = mat_cal.distance_BetweenZeroPointAndPoint2D(input[i].center);
    if(distance < min_distance)
    {
      return_line.init();
      break;
    }
  }

  return return_line;
}

eun_u::Point3D social_recharge::cal_tarket_position(eun_u::Line2D input)
{
  int num_point = (int) input.points.size();
  eun_u::Point3D target;
  eun_u::Vector3D direction;

  //Line의 각도 구하기(로봇과 충전기간의 틀어진 각도)
  if(input.points[0].y > input.points[num_point - 1].y)
  {
    direction.X = input.points[0].x - input.points[num_point - 1].x;
    direction.Y = input.points[0].y - input.points[num_point - 1].y;
  }

  else
  {
    direction.X = input.points[num_point - 1].x - input.points[0].x;
    direction.Y = input.points[num_point - 1].y - input.points[0].y;
  }

  direction.Z = 0;
  direction = mat_cal.make_unitvector(direction);
  double angle = mat_cal.angle_BetweenAxisXandAvector(direction);

  if(direction.Y < 0)
  {
    angle = 90 - fabs(angle);
  }
  else
  {
    angle = -(90 - fabs(angle));
  }

  //충전기의 위치
  target.x = input.center.x;
  target.y = input.center.y;
  target.z = angle;

  //Target information
  printf("Target Dis : %0.3lf\tX : %0.3lf\tY : %0.3lf\tAngle : %0.3lf\n", input.distance, target.x, target.y, target.z);
  return target;
}

void social_recharge::wheel_command_laser_rough(eun_u::Point3D position)
{
  eun_u::Point2D pose;
  double angle, distance;
  geometry_msgs::Twist msg;
  msg.linear.x = msg.linear.y = msg.linear.z = 0;
  msg.angular.x = msg.angular.y = msg.angular.z = 0;

  pose.x = position.x;
  pose.y = position.y;
  angle = position.z;

  if(fabs(angle) > m_angle_threshold )
  {
    if(angle < 0) msg.angular.z = -0.3;
    else msg.angular.z = 0.3;
  }

  else if(fabs(pose.y) > m_trans_threshold * 2)
  {
    if(pose.y > 0) msg.linear.y = -0.1;
    else msg.linear.y = 0.1;
  }

  else if(fabs(pose.x) > m_trans_threshold * 2)
  {
    msg.linear.x = -0.1;
  }

  printf("position X : %0.3lf\tY : %0.3lf\tZ : %0.3lf\n", pose.x, pose.y, angle);
  cout << "Wheel X : " << msg.linear.x << " Y : " << msg.linear.y << " Z : " << msg.angular.z << endl;
  //printf("Wheel X : %0.3f\tY : %0.3f\tZ : %0.3f\n", msg.linear.x, msg.linear.y, msg.linear.z);
  m_pub_wheel.publish(msg);
}

void social_recharge::wheel_command_laser_detail(eun_u::Point3D position)
{
  eun_u::Point2D pose;
  double angle, distance;
  geometry_msgs::Twist msg;
  msg.linear.x = msg.linear.y = msg.linear.z = 0;
  msg.angular.x = msg.angular.y = msg.angular.z = 0;

  pose.x = position.x;
  pose.y = position.y;
  angle = position.z;

  if (m_charge == true)
  {
    msg.linear.x = msg.linear.y = msg.linear.z = 0;
    msg.angular.x = msg.angular.y = msg.angular.z = 0;
  }

  else {

    if(fabs(angle) > 2.0)
    {
      if(angle < 0) msg.angular.z = -0.05;
      else msg.angular.z = 0.05;
    }

    else if(fabs(pose.y) > m_trans_threshold)
    {
      if(pose.y > 0) msg.linear.y = -0.03;
      else msg.linear.y = 0.03;
    }

    else if(fabs(pose.x) > m_trans_threshold)
    {
      msg.linear.x = - 0.03;
    }

  }

  printf("position X : %0.3lf\tY : %0.3lf\tZ : %0.3lf\n", pose.x, pose.y, angle);
  cout << "Wheel X : " << msg.linear.x << " Y : " << msg.linear.y << " Z : " << msg.angular.z << endl;
  //printf("Wheel X : %0.3f\tY : %0.3f\tZ : %0.3f\n", msg.linear.x, msg.linear.y, msg.linear.z);
  m_pub_wheel.publish(msg);
}

void social_recharge::rotation(double angle)
{
  double target_angle = m_pose.z + angle;
  if(target_angle > 180) target_angle = target_angle - 360;
  else if(target_angle < -180) target_angle = target_angle + 360;

  printf("Target Angle : %0.3lf\tStart Angle : %0.3lf\n", target_angle, m_pose.z);

  double bet_angle;
  ros::Rate r(9);
  while(ros::ok())
  {
    geometry_msgs::Twist msg;
    msg.linear.x = msg.linear.y = msg.linear.z = 0;
    msg.angular.x = msg.angular.y = msg.angular.z = 0;

    bet_angle = target_angle - m_pose.z;
    if(bet_angle > 180) bet_angle = bet_angle - 360;
    else if(bet_angle < -180) bet_angle = bet_angle + 360;

    printf("Target Angle : %0.3lf\tCur Angle : %0.3lf\n", target_angle, m_pose.z);
    if(fabs(bet_angle) > 30)
    {
      //if(bet_angle > 0)  msg.angular.z = 1.0;
      //else msg.angular.z = -1.0;
      msg.angular.z = 1.0;

      m_pub_wheel.publish(msg);
    }

    else if(fabs(bet_angle) > 5)
    {
      //if(bet_angle > 0)  msg.angular.z = 0.05;
      //else msg.angular.z = -0.05;
      msg.angular.z = 0.2;
      m_pub_wheel.publish(msg);
    }

    else
    {
      m_pub_wheel.publish(msg);
      break;
    }
    r.sleep();
  }
}

void social_recharge::wheelmoveX(double distance, double speed)
{
  double point_distance;
  double target_distance;
  eun_u::Point3D robot_pose;
  eun_u::Point3D start_pose,target_pose;
  start_pose.x = m_pose.x;
  start_pose.y = m_pose.y;
  robot_pose.z = 0;

  ros::Rate r(9);
  while(ros::ok())
  {
    bool stop_flag = false;
    geometry_msgs::Twist msg;
    msg.linear.x = msg.linear.y = msg.linear.z = 0;
    msg.angular.x = msg.angular.y = msg.angular.z = 0;

    robot_pose.x = m_pose.x;
    robot_pose.y = m_pose.y;
    robot_pose.z = 0;

    for(int i = 0; i < (int) m_laser.size(); i++)
    {
      point_distance = mat_cal.distance_BetweenZeroPointAndPoint2D(m_laser[i]);
      if(point_distance != 0 && point_distance < 0.10) stop_flag = true;
    }

    target_distance = mat_cal.distance_BetweenPoint3DAndPoint3D(robot_pose, start_pose);
    printf("Target_Dis : %0.3lf\tMove_Dis : %0.3lf\n", distance, target_distance);

    if(target_distance > fabs(distance))
    {
      m_pub_wheel.publish(msg);
      break;
    }

    else if(stop_flag == true) m_pub_wheel.publish(msg);

    else
    {
      if(distance > 0) msg.linear.x = fabs(speed);
      else msg.linear.x = -fabs(speed);
      m_pub_wheel.publish(msg);
    }

    r.sleep();
  }
}

void social_recharge::line_display(eun_u::Line2D tarket, vector<eun_u::Line2D> line)
{
  visualization_msgs::Marker points;
  visualization_msgs::Marker tarket_line, general_line1, general_line2, general_line3;

  points.header.frame_id = tarket_line.header.frame_id = "/laser";
  general_line1.header.frame_id = general_line2.header.frame_id = general_line3.header.frame_id = "/laser";

  points.header.stamp = tarket_line.header.stamp = ros::Time::now();
  general_line1.header.stamp = general_line2.header.stamp = general_line3.header.stamp = ros::Time::now();

  points.ns = "Points";
  tarket_line.ns = general_line1.ns = general_line2.ns = general_line3.ns ="Laser";

  points.action = tarket_line.action = visualization_msgs::Marker::ADD;
  general_line1.action = general_line2.action = general_line3.action = visualization_msgs::Marker::ADD;

  points.pose.orientation.w = tarket_line.pose.orientation.w = 1.0;
  general_line1.pose.orientation.w = general_line2.pose.orientation.w = general_line3.pose.orientation.w = 1.0;

  points.type = visualization_msgs::Marker::POINTS;
  tarket_line.type = general_line1.type = general_line2.type = general_line3.type = visualization_msgs::Marker::POINTS;

  points.scale.x = points.scale.y = points.scale.z = 0.03;
  tarket_line.scale.x = tarket_line.scale.y = tarket_line.scale.z = 0.03;
  general_line1.scale.x = general_line1.scale.y = general_line1.scale.z = 0.03;
  general_line2.scale.x = general_line2.scale.y = general_line2.scale.z = 0.03;
  general_line3.scale.x = general_line3.scale.y = general_line3.scale.z = 0.03;

  points.id = 1;
  tarket_line.id = 2;
  general_line1.id = 3;
  general_line2.id = 4;
  general_line3.id = 5;

  points.color.r = 0.0f;
  points.color.g = 1.0f;
  points.color.b = 0.0f;

  tarket_line.color.r = 1.0f;
  tarket_line.color.g = 0.0f;
  tarket_line.color.b = 0.0f;

  general_line1.color.r = 0.0f;
  general_line1.color.g = 1.0f;
  general_line1.color.b = 0.0f;

  general_line2.color.r = 0.0f;
  general_line2.color.g = 0.0f;
  general_line2.color.b = 1.0f;

  general_line3.color.r = 1.0f;
  general_line3.color.g = 1.0f;
  general_line3.color.b = 1.0f;

  points.color.a = tarket_line.color.a = 1.0;
  general_line1.color.a = general_line2.color.a = general_line3.color.a = 1.0;

  geometry_msgs::Point point;
  point.z = 0.3;

  if((int) m_laser.size() > 0)
  {
    for(int i = 0; i < (int) m_laser.size(); i++)
    {
      point.x = m_laser[i].x;
      point.y = m_laser[i].y;
      points.points.push_back(point);
    }
    m_pub_marker.publish(points);
  }

  if((int) tarket.points.size() > 0)
  {
    for(int i = 0; i < (int) tarket.points.size(); i++)
    {
      point.x = tarket.points[i].x;
      point.y = tarket.points[i].y;
      tarket_line.points.push_back(point);
    }
    m_pub_marker.publish(tarket_line);
  }

  if((int) line.size() > 0)
  {
    for(int i = 0; i < (int) line.size(); i++)
    {
      for(int j = 0; j < (int) line[i].points.size(); j++)
      {
        point.x = line[i].points[j].x;
        point.y = line[i].points[j].y;
        if(i%3 == 0)	general_line1.points.push_back(point);
        else if(i%3 == 1)	general_line2.points.push_back(point);
        else	general_line3.points.push_back(point);
      }
    }

    m_pub_marker.publish(general_line1);
    ros::Duration(0.01).sleep();
    m_pub_marker.publish(general_line2);
    ros::Duration(0.01).sleep();
    m_pub_marker.publish(general_line3);
    ros::Duration(0.01).sleep();
  }
}

int main(int argc, char** argv)
{
  cout << "Start auto_parking" <<endl;
  ros::init(argc, argv, "social_recharge_node");

  social_recharge n_t;

  ros::MultiThreadedSpinner spinner(4); // Use 4 threads
  spinner.spin(); // spin() will not return until the node has been shutdown

  return 0;
}
