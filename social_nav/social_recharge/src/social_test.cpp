
#include "social_test.h"
#include <math.h>

// init
social_test::social_test()
{
    //기본 값 init

    m_pose.init();


//    pthread_t thread_key;
//    pthread_create(&thread_key, NULL, social_test::thread_key_input, this);

    //printf("trans_threshold : %0.3lf\tangle_threshold : %0.3lf\n", m_trans_threshold, m_angle_threshold);
    //printf("Min Y : %0.3lf\tma Y : %0.3lf\n", m_laser_area.min_y, m_laser_area.max_y);

    init_node();
}

social_test::~social_test()
{

}

void social_test::init_node()
{
    //Navigation
    // is docking check
    //    m_sub_odom = m_nodeHandle.subscribe("/odom", 1, &social_test::odom_SubscribeCallBack, this);
    m_ser_clearCostmap = m_nodeHandle.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

    m_pub_goal = m_nodeHandle.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1000);
    m_ptr_pub = m_nodeHandle.advertise<geometry_msgs::PoseStamped> ("/robocare/navigation/go_to_goal", 1000);
    m_pub_wheel = m_nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    m_sub_get_pose = m_nodeHandle.subscribe("/amcl_pose", 1, &social_test::get_pose_SubscribeCallBack, this);
    m_sub_pose = m_nodeHandle.subscribe("/odom", 1, &social_test::pose_SubscribeCallBack, this);
    m_sub_init_button = m_nodeHandle.subscribe("/social_nav/go_to_init", 1, &social_test::go_to_init_SubscribeCallBack, this);
    m_sub_goal_button = m_nodeHandle.subscribe("/social_nav/go_to_goal", 1, &social_test::go_to_goal_SubscribeCallBack, this);
}

/***********************
   CallBack Funtion
************************/

void social_test::pose_SubscribeCallBack(const nav_msgs::Odometry::ConstPtr& input)
{
    m_pose.x = input->pose.pose.position.x;
    m_pose.y = input->pose.pose.position.y;
    m_pose.z = input->pose.pose.orientation.z;
}


void social_test::get_pose_SubscribeCallBack(geometry_msgs::PoseWithCovarianceStamped input)
{
    m_get_pose.x = input.pose.pose.position.x;
    m_get_pose.y = input.pose.pose.position.y;
    m_get_pose.z = input.pose.pose.orientation.z;
    m_get_pose.w = input.pose.pose.orientation.w;
}

void social_test::go_to_init_SubscribeCallBack(const std_msgs::Bool& input)
{
  if(input.data == true)
  {
    go_to_goal(72.2302856445, 4.71917533875 , 0.712238204224, 0.701937846567);
  }

}

void social_test::go_to_goal_SubscribeCallBack(const std_msgs::Bool& input)
{
  if(input.data == true)
  {
    go_to_goal(72.0487289429, 9.44694519043, -0.700640609546, 0.713514356026);
  }
}

//void* social_test::thread_key_input(void *arg)
//{
//    ((social_test*)(arg))->key_input();
//    return 0;
//}

//int social_test::getch(){

//    struct termios oldt, newt;
//    int ch;

//    tcgetattr( STDIN_FILENO, &oldt );
//    newt = oldt;
//    newt.c_lflag &= ~(ICANON | ECHO);
//    newt.c_cc[VMIN] = 0;
//    newt.c_cc[VTIME] = 1;
//    tcsetattr( STDIN_FILENO, TCSANOW, &newt );
//    ch = getchar();
//    tcsetattr( STDIN_FILENO, TCSANOW, &oldt );

//    return ch;

//}

//int social_test::kbhit(){

//    struct termios oldt, newt;
//    int ch;
//    int oldf;

//    tcgetattr(STDIN_FILENO, &oldt);
//    newt = oldt;
//    newt.c_lflag &= ~(ICANON | ECHO);
//    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
//    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
//    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

//    ch = getchar();

//    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
//    fcntl(STDIN_FILENO, F_SETFL, oldf);

//    if (ch != EOF)
//    {
//        ungetc(ch, stdin);
//        return 1;
//    }
//    return 0;
//}


/*****************************************************************************
 ** Key Input Function
 *****************************************************************************/
//void social_test::key_input(){

//    std::string msg =

//            "\n\
//            Test Docking & Driving Your Mobile Robot! \n\
//            --------------------------- \n\
//            Input Key:\n\
//            q  w  e\n\
//            s  d\n\
//            x  c \n\
//            \n\
//            q : Roop exit !!\n\
//            w : Leaving station !!\n\
//            e : Docking Test\n\
//            s : Force Stop !!!!\n\
//            d : Driving & Docking Test !!\n\
//            x : Map MODE \n\
//            c : Map SAVE \n\
//            \n\
//            CTRL-C to quit\n\
//            ";

//            ROS_INFO("%s", msg.c_str());

//    ros::Rate loop_rate(19);

//    while(ros::ok())
//    {
//        if (kbhit())
//        {
//            char c = getch();

//            if (c == GO_50_CM)   wheel_forward_test(0.5);
//            else if (c == GO_100_CM)
//            {

//                go_to_goal(72.2302856445, 4.71917533875 , 0.712238204224, 0.701937846567);

//            }
//            else if (c == GO_300_CM)
//            {
//                go_to_goal(72.0487289429, 9.44694519043, -0.700640609546, 0.713514356026);
//            }
//            else if (c == BACK_HOME)  wheel_backward_test(0.0);
//            else if (c == STOP)
//            {

//                pthread_t thread_stopping;
//                pthread_create(&thread_stopping, NULL, social_test::thread_stopping_test, this);
//            }

//        }
//        ros::spinOnce();
//        loop_rate.sleep();
//    }
//}




void social_test::go_to_goal(double x, double y, double angle, double w)
{

    printf("Set Goal!\n");
    std_srvs::Empty em;                                                                             // Empty
    m_ser_clearCostmap.call(em);
    ros::Duration(0.5).sleep();

    m_goal.goal.target_pose.header.frame_id = "map";                                                // goal frame_id
    m_goal.goal.target_pose.header.stamp = ros::Time::now();                                        //      time

    m_goal.goal.target_pose.pose.position.x = x;                       //      position [x, y, z]
    m_goal.goal.target_pose.pose.position.y = y;
    m_goal.goal.target_pose.pose.orientation.z = angle;
    m_goal.goal.target_pose.pose.orientation.w = w;

    m_pub_goal.publish(m_goal);

}


void social_test::wheel_forward_test(double distance)
{
    geometry_msgs::Twist msg;
    msg.linear.x = 0.2;
    msg.linear.y = msg.linear.z = 0;

    int cnt = 0;

    ros::Rate r(9);
    while(ros::ok())
    {
        if( m_pose.x > distance) {
            msg.linear.x = 0.0;
            msg.linear.y = msg.linear.z = 0;
            m_pub_wheel.publish(msg);
            break;
        }

        m_pub_wheel.publish(msg);
        r.sleep();
    }

}

void social_test::wheel_backward_test(double distance)
{
    geometry_msgs::Twist msg;
    msg.linear.x = -0.2;
    msg.linear.y = msg.linear.z = 0;

    int cnt = 0;

    ros::Rate r(9);
    while(ros::ok())
    {
        if( m_pose.x < distance) {
            msg.linear.x = 0.0;
            msg.linear.y = msg.linear.z = 0;
            m_pub_wheel.publish(msg);
            break;
        }
        m_pub_wheel.publish(msg);
        r.sleep();
    }
}

void* social_test::thread_stopping_test(void *arg)
{
    ((social_test*)(arg))->wheel_stop();
    return 0;
}

void social_test::wheel_stop()
{
    ros::Rate r(9);
    while(ros::ok())
    {
        geometry_msgs::Twist msg;
        msg.linear.x = 0.0;
        msg.linear.y = msg.linear.z = 0;
        m_pub_wheel.publish(msg);
        r.sleep();
    }
}

void social_test::rotation(double angle)
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





int main(int argc, char** argv)
{
    cout << "Start navigation_test" <<endl;
    ros::init(argc, argv, "social_drive_node");

    social_test n_t;

    ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    spinner.spin(); // spin() will not return until the node has been shutdown

    return 0;
}
