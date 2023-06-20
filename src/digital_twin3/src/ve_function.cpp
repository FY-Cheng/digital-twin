#include "ve_configure.h"


// Add obstacles in my world.
std::string RosBasic::importObstacle(double x, double y, double r)
{
    std::string translation = "translation "+std::to_string(x)+" "+std::to_string(y)+" 0.105";
    std::string color = "color 0.937255 0.160784 0.160784";
    std::string radius = "radius "+std::to_string(r);
    std::string str = "Obstacle {"+translation+", "+color+", "+radius+"}";
    // example: "Obstacle {translation 0.05 0.02 0.025, color 0.937255 0.160784 0.160784, radius 0.04}";
    return str;
}

bool RosBasic::sendRobotEnvCallback(digital_twin3::get_robot_env::Request& req, digital_twin3::get_robot_env::Response& res)// 
{
    // Supervisor

    // uint64_t worldNode = supervisor.supervisor_get_world(); // get world.
    // uint64_t worldChildrenField = supervisor.supervisor_node_get_field(worldNode,"children"); // get children.
    
    uint64_t robotArenaNode = supervisor.supervisor_get_form_DEF("robotArena");
    uint64_t floorSizeField = supervisor.supervisor_node_get_field(robotArenaNode,"floorSize");
    geometry_msgs::Vector3 robotArenaSize = supervisor.supervisor_field_get_vec2f(floorSizeField,-1);
    res.x_size = robotArenaSize.x;
    res.y_size = robotArenaSize.y;
    ROS_INFO("getWebotsEnvService | Size of robot arena: X:%.2f, Y:%.2f.",res.x_size,res.y_size);

    uint64_t myRobotNode = this->supervisor.supervisor_get_form_DEF("myRobot");
    res.robot_pos = this->supervisor.supervisor_node_get_position(myRobotNode);
    ROS_INFO("getWebotsEnvService | Current robot position: X:%.2f, Y:%.2f.",res.robot_pos.x,res.robot_pos.y);

    uint64_t endFlagNode = this->supervisor.supervisor_get_form_DEF("endFlag");
    res.end_pos = this->supervisor.supervisor_node_get_position(endFlagNode);
    ROS_INFO("getWebotsEnvService | End flag position: X:%.2f, Y:%.2f.",res.end_pos.x,res.end_pos.y);

    uint64_t obstaclesGroupNode = this->supervisor.supervisor_get_form_DEF("staticObstacles");
    uint64_t obstaclesChildrenField = this->supervisor.supervisor_node_get_field(obstaclesGroupNode,"children");
    int32_t static_obstacles_count = this->supervisor.supervisor_field_get_count(obstaclesChildrenField);
    ROS_INFO("getWebotsEnvService | Static obstacles group has %d obstalces",static_obstacles_count);

    for (int i = 0; i < static_obstacles_count; ++i)
    {
        uint64_t staticObsNode = this->supervisor.supervisor_field_get_node(obstaclesChildrenField,i);
        res.static_obs.push_back(this->supervisor.supervisor_node_get_position(staticObsNode));
    }

    return true;
    
}

void RosBasic::Webots_robot()
{
    ros::ServiceClient W1SetVelocityClient = n.serviceClient<webots_ros::set_float>(controllerName + "/wheel1/set_velocity");
    ros::ServiceClient W2SetVelocityClient = n.serviceClient<webots_ros::set_float>(controllerName + "/wheel2/set_velocity");
    ros::ServiceClient W3SetVelocityClient = n.serviceClient<webots_ros::set_float>(controllerName + "/wheel3/set_velocity");
    ros::ServiceClient W4SetVelocityClient = n.serviceClient<webots_ros::set_float>(controllerName + "/wheel4/set_velocity");
    webots_ros::set_float W1SetVelocitySrv;
    webots_ros::set_float W2SetVelocitySrv;
    webots_ros::set_float W3SetVelocitySrv;
    webots_ros::set_float W4SetVelocitySrv;

    ros::ServiceClient Wheel1SetPositionClient = n.serviceClient<webots_ros::set_float>(controllerName + "/wheel1/set_position");
    ros::ServiceClient Wheel2SetPositionClient = n.serviceClient<webots_ros::set_float>(controllerName + "/wheel2/set_position");
    ros::ServiceClient Wheel3SetPositionClient = n.serviceClient<webots_ros::set_float>(controllerName + "/wheel3/set_position");
    ros::ServiceClient Wheel4SetPositionClient = n.serviceClient<webots_ros::set_float>(controllerName + "/wheel4/set_position");
    webots_ros::set_float wheel1Srv;
    webots_ros::set_float wheel2Srv;
    webots_ros::set_float wheel3Srv;
    webots_ros::set_float wheel4Srv;
    
    // set motor to velocity control
    // sleep(3);
    wheel1Srv.request.value = INFINITY;
    wheel2Srv.request.value = INFINITY;
    wheel3Srv.request.value = INFINITY;
    wheel4Srv.request.value = INFINITY;
    while (1)
    {
        if (Wheel1SetPositionClient.call(wheel1Srv) &&
            Wheel2SetPositionClient.call(wheel2Srv) &&
            Wheel3SetPositionClient.call(wheel3Srv) &&
            Wheel4SetPositionClient.call(wheel4Srv))
            {
                ROS_INFO("Weobts sets motors to velocity control mode.");
                break;
            }
            
    }

    // if (!Wheel1SetPositionClient.call(wheel1Srv) ||
    //     !Wheel2SetPositionClient.call(wheel2Srv) ||
    //     !Wheel3SetPositionClient.call(wheel3Srv) ||
    //     !Wheel4SetPositionClient.call(wheel4Srv))
    //     ROS_WARN("Weobts sets motors to velocity control mode failed.");

    W1SetVelocitySrv.request.value = 0;
    W2SetVelocitySrv.request.value = 0;
    W3SetVelocitySrv.request.value = 0;
    W4SetVelocitySrv.request.value = 0;

    if (!W1SetVelocityClient.call(W1SetVelocitySrv) ||
        !W2SetVelocityClient.call(W2SetVelocitySrv) ||
        !W3SetVelocityClient.call(W3SetVelocitySrv) ||
        !W4SetVelocityClient.call(W4SetVelocitySrv) ||
        !W1SetVelocitySrv.response.success ||
        !W2SetVelocitySrv.response.success ||
        !W3SetVelocitySrv.response.success ||
        !W4SetVelocitySrv.response.success)
        ROS_WARN("Motors of dt_robot initialization failed.");


    // subscribe
    ros::Subscriber GPSValueSub;
    ros::Subscriber ImuValueSub;
    ros::Subscriber Controller2Sub;

    Controller2Sub = n.subscribe("/controller2",1,&RosBasic::controller2Callback, this);

    // enable GPS
    ros::ServiceClient enableGPSClient = n.serviceClient<webots_ros::set_int>(controllerName + "/gps/enable");
    webots_ros::set_int enableGPSSrv;
    enableGPSSrv.request.value = TIME_STEP;
    if (enableGPSClient.call(enableGPSSrv) && enableGPSSrv.response.success)
    {
        ROS_INFO("GPS enabled.");
        GPSValueSub = n.subscribe(controllerName + "/gps/values", 1, &RosBasic::GPSCallback, this);
    }
    
    // enable imu
    ros::ServiceClient enableImuClient = n.serviceClient<webots_ros::set_int>(controllerName + "/inertial_unit/enable");
    webots_ros::set_int enableImuSrv;
    enableImuSrv.request.value = TIME_STEP;
    if (enableImuClient.call(enableImuSrv) && enableImuSrv.response.success)
    {
        ROS_INFO("Imu enabled.");
        ImuValueSub = n.subscribe(controllerName + "/inertial_unit/quaternion", 1, &RosBasic::imuCallback, this);
    }

    // publisher
    ros::Publisher odom_pub  = n.advertise<nav_msgs::Odometry>(controllerName + "/odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    // service
    ros::ServiceServer getRobotEnvService = 
        n.advertiseService("get_robot_env_infor", &RosBasic::sendRobotEnvCallback,this);
    
    Supervisor supervisor1(n,controllerName);
    uint64_t worldNode = supervisor1.supervisor_get_world();

    // main loop
    ros::Rate loop_rate(1 / TRACK_TIME_STEP);
    while (ros::ok())
    {
        ros::spinOnce();
        if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success)
        {
            ROS_ERROR("Failed to call service time_step for next step.");
        }
        
        // int hasImport = supervisor.field_import_node_from_string(obstaclesChildrenField,-1,importObstacle(3,3,0.5));

        // Velocity driven.
        this->WheelVelocity[0] = 1 / WHEEL_RADIUS * (q_vx + q_vy + (LX + LY) * q_vth);
        this->WheelVelocity[1] = 1 / WHEEL_RADIUS * (q_vx - q_vy - (LX + LY) * q_vth);
        this->WheelVelocity[2] = 1 / WHEEL_RADIUS * (q_vx - q_vy + (LX + LY) * q_vth);
        this->WheelVelocity[3] = 1 / WHEEL_RADIUS * (q_vx + q_vy - (LX + LY) * q_vth);

        W1SetVelocitySrv.request.value = this->WheelVelocity[0];
        W2SetVelocitySrv.request.value = this->WheelVelocity[1];
        W3SetVelocitySrv.request.value = this->WheelVelocity[2];
        W4SetVelocitySrv.request.value = this->WheelVelocity[3];

        if (!W1SetVelocityClient.call(W1SetVelocitySrv) ||
            !W2SetVelocityClient.call(W2SetVelocitySrv) ||
            !W3SetVelocityClient.call(W3SetVelocitySrv) ||
            !W4SetVelocityClient.call(W4SetVelocitySrv) ||
            !W1SetVelocitySrv.response.success ||
            !W2SetVelocitySrv.response.success ||
            !W3SetVelocitySrv.response.success ||
            !W4SetVelocitySrv.response.success)
            ROS_ERROR("Failed to send new position commands to the robot.");

        this->vx = this->q_vx;
        this->vy = this->q_vy;
        this->vth = this->q_vth;


        // Topic publish
        // Publishing Odometry
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(this->th);

        // first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = this->x;
        odom_trans.transform.translation.y = this->y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        // send the transform
        odom_broadcaster.sendTransform(odom_trans);

        // next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";

        // set the position
        odom.pose.pose.position.x = this->x;
        odom.pose.pose.position.y = this->y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        // set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = this->vx;
        odom.twist.twist.linear.y = this->vy;
        odom.twist.twist.angular.z = this->vth;

        // publish the message
        odom_pub.publish(odom);

        loop_rate.sleep();
    }
}



RosBasic::RosBasic(ros::NodeHandle &nh) : supervisor(nh, "dt_robot") // Initialize supervisor
{
    this->n = nh;
    this->controllerCount = 0;
    ros::Subscriber nameSub = n.subscribe("model_name", 100, &RosBasic::controllerNameCallback, this);
    while (this->controllerCount == 0 || this->controllerCount < nameSub.getNumPublishers())
    {
        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();
    }
    ros::spinOnce();

    // if there is more than one controller available, let the user choose
    if (this->controllerCount == 1)
        this->controllerName = this->controllerList[0];
    else
    {
        int wantedController = 0;
        std::cout << "Choose the # of the controller you want to use:\n";
        std::cin >> wantedController;
        if (1 <= wantedController && wantedController <= this->controllerCount)
            this->controllerName = this->controllerList[wantedController - 1];
        else
        {
            ROS_ERROR("Invalid number for controller choice.");
        }
    }
    // std::cout<<(controllerName);

    // leave topic once it is not necessary anymore
    nameSub.shutdown();

    timeStepClient = 
        n.serviceClient<webots_ros::set_int>(this->controllerName + "/robot/time_step");
    timeStepSrv.request.value = TIME_STEP;
}

RosBasic::~RosBasic()
{
    timeStepSrv.request.value = 0;
    timeStepClient.call(timeStepSrv);
}

double RosBasic::Limit_angular(double th)
{
    if (th > PI)
    {
        th -= 2*PI;
    }
    if (th < -PI)
    {
        th += 2*PI;
    }
    return th;
}

void quit(int sig)
{
    ROS_INFO("User stopped the 'virtual_entity' node.");
    ros::shutdown();
    exit(0);
}

void RosBasic::ros_quit()
{
    timeStepSrv.request.value = 0;
    timeStepClient.call(timeStepSrv);
    ros::shutdown();
}



//  Callback Function
void RosBasic::controllerNameCallback(const std_msgs::String::ConstPtr &name)
{
    this->controllerCount++;
    this->controllerList.push_back(name->data);
    ROS_INFO("Controller #%d: %s.", this->controllerCount, this->controllerList.back().c_str());
}

void RosBasic::GPSCallback(const geometry_msgs::PointStamped::ConstPtr &values)
{
    this->x = values->point.x;
    this->y = values->point.y;
}

void RosBasic::imuCallback(const sensor_msgs::Imu::ConstPtr &values)
{
    double Ox = values->orientation.x;
    double Oy = values->orientation.y;
    double Oz = values->orientation.z;
    double Ow = values->orientation.w;

    // Quaternion to EulerAngle
    double siny_cosp = 2 * (Ow * Oz + Ox * Oy);
    double cosy_cosp = 1 - 2 * (Oy * Oy + Oz * Oz);
    this->th = std::atan2(siny_cosp, cosy_cosp);
    // ROS_INFO("yaw = %.7f",this->th);
}

void RosBasic::controller2Callback(const geometry_msgs::Point::ConstPtr &values)
{
    this->q_vx = values->x;
    this->q_vy = values->y;
    this->q_vth = values->z;
}