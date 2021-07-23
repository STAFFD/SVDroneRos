#ifndef DRONE_MONITOR_H
#define DRONE_MONITOR_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <serial/serial.h>

using namespace std;

class DroneMonitor{
private:
public:
    DroneMonitor();
    ~DroneMonitor();
};


#endif