#ifndef DRONE_MONITOR_H
#define DRONE_MONITOR_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>

// #define DEBUG_PRINT

#define TRANS_BIAS              -2

#define TRANS_START_HIGH		0
#define TRANS_START_LOW			1

#define TRANS_GYRO_ROLL_HIGH	2+TRANS_BIAS
#define TRANS_GYRO_ROLL_LOW		3+TRANS_BIAS
#define TRANS_GYRO_PITCH_HIGH	4+TRANS_BIAS
#define TRANS_GYRO_PITCH_LOW	5+TRANS_BIAS
#define TRANS_GYRO_YAW_HIGH		6+TRANS_BIAS
#define TRANS_GYRO_YAW_LOW		7+TRANS_BIAS

#define TRANS_ANG_ROLL_HIGH		8+TRANS_BIAS
#define TRANS_ANG_ROLL_LOW		9+TRANS_BIAS
#define TRANS_ANG_PITCH_HIGH	10+TRANS_BIAS
#define TRANS_ANG_PITCH_LOW		11+TRANS_BIAS
#define TRANS_ANG_YAW_HIGH		12+TRANS_BIAS
#define TRANS_ANG_YAW_LOW		13+TRANS_BIAS

#define TRANS_KAL_ROLL_HIGH		14+TRANS_BIAS
#define TRANS_KAL_ROLL_LOW		15+TRANS_BIAS
#define TRANS_KAL_PITCH_HIGH	16+TRANS_BIAS
#define TRANS_KAL_PITCH_LOW		17+TRANS_BIAS
#define TRANS_TOTAL             TRANS_KAL_PITCH_LOW+1

using namespace std;

typedef struct{
    float angle_roll;
    float angle_pitch;
    float angle_yaw;
    float gyro_roll;
    float gyro_pitch;
    float gyro_yaw;

    float kal_pitch;
    float kal_roll;
}DroneData;

class DroneMonitor{
private:
    ros::NodeHandle nh;
    serial::Serial ser;
    ros::Publisher pub_;
    ros::Publisher pub_kal;
    uint8_t resultBuffer[TRANS_TOTAL];
    DroneData droneData;
    sensor_msgs::Imu imu;
    sensor_msgs::Imu imu_kal;
public:
    DroneMonitor();
    void run();
    ~DroneMonitor();
};


#endif