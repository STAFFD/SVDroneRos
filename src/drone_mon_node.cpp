#include "drone_mon.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "drone_mon");
    DroneMonitor droneMon;
    droneMon.run();
    ros::spin();
    return 0;
}
