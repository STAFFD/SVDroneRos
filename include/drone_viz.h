#ifndef DRONE_VIZ_H
#define DRONE_VIZ_H

#include <ros/ros.h>
#include <std_msgs/String.h>
// #include <robot_state_publisher/robot_state_publisher.h>
// #include <kdl_parser/kdl_parser.hpp>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
// #include <tf/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>

using namespace std;

#define DEGREE2RAD 3.1415926/180

class DroneViz{
private:
    ros::NodeHandle n;
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    ros::Publisher joint_pub;
    ros::Subscriber sub_;
    tf::TransformBroadcaster broadcaster;
    // robot_state_publisher::RobotStatePublisher rsPub;
public:
    DroneViz(/* args */);
    void IMUCallback(const sensor_msgs::Imu imuData);
    ~DroneViz();
};


#endif