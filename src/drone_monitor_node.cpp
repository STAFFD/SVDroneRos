#include "drone_monitor.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "SVDroneMonitor");
    DroneMonitor droneMon;
    ros::spin();
    return 0;
}
