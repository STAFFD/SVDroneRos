#include "drone_monitor.h"

DroneMonitor::DroneMonitor(/* args */){
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);
    serial::Serial ser;
    try{
        ser.setPort("/dev/tty.usbmodem3372364D30341");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e){
        ROS_ERROR_STREAM("Unable to open port ");
        // exit();
    }
    while(ros::ok()){
        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            result.data = ser.read(ser.available());
            ROS_INFO_STREAM("Read: " << result.data);
            // read_pub.publish(result);
        }
        loop_rate.sleep();
    }
}

DroneMonitor::~DroneMonitor()
{
}