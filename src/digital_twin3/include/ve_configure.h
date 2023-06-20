#include <signal.h>
#include <stdio.h>
#include "ros/ros.h"
#include <unistd.h>
#include <limits>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <webots_ros/Int32Stamped.h>
#include <webots_ros/Float64Stamped.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/get_int.h>
#include <webots_ros/robot_get_device_list.h>

#include <webots_ros/node_get_field.h>
#include <webots_ros/get_uint64.h>
#include <webots_ros/field_import_node_from_string.h>
#include <webots_ros/field_import_node.h>

#include "supervisor.h"
#include "digital_twin3/get_robot_env.h"

#define MOTOR_POSI_ADD 0.3
#define WHEEL_RADIUS 0.05
#define LX 0.156    // half axlespacing, longitudinal distance from robot's COM to wheel [m].
#define LY 0.176    // half wheelspacing, lateral distance from robot's COM to wheel [m].
#define TRACK_TIME_STEP 0.1
#define PI 3.14159265358
#define TIME_STEP 32;

void quit(int sig);


// Class for co-simulation of Webots and ROS
class RosBasic
{
    ros::NodeHandle n;
    std::string controllerName;
    std::string nodeName;
    int controllerCount;
    int controlMode;
    std::vector<std::string> controllerList;
    Supervisor supervisor;

    double w1Position = 0;
    double w2Position = 0;
    double w3Position = 0;
    double w4Position = 0;
    double GPSValues[3];
    double WheelVelocity[4];

    double x;
    double y;
    double th;
    double vx;
    double vy;
    double vth;

    double q_vx;
    double q_vy;
    double q_vth;

    ros::ServiceClient timeStepClient;
    webots_ros::set_int timeStepSrv;

public:
    RosBasic(ros::NodeHandle &nh);
    ~RosBasic();

    void ros_quit();
    void Webots_robot();
    double Limit_angular(double th);
    std::string importObstacle(double x, double y, double radius);

    bool sendRobotEnvCallback(digital_twin3::get_robot_env::Request &req, digital_twin3::get_robot_env::Response &res);
    void controllerNameCallback(const std_msgs::String::ConstPtr &name);
    void GPSCallback(const geometry_msgs::PointStamped::ConstPtr &values);
    void imuCallback(const sensor_msgs::Imu::ConstPtr &values);
    void controller2Callback(const geometry_msgs::Point::ConstPtr &values);

};
