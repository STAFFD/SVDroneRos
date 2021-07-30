#include "drone_viz.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "drone_viz");
    DroneViz droneViz;
    ros::spin();
    return 0;
}
