#include "drone_viz.h"

DroneViz::DroneViz(/* args */){
    // KDL::Tree droneModelTree;
    // urdf::Model droneModel;
    // string droneModelPath;
    // ros::param::get(ros::this_node::getName()+"/usb_port", 
    // droneModelPath);

    // TiXmlDocument xml_doc;
    // xml_doc.Parse(droneModelPath.c_str());

    // if (!droneModel.initXml(&xml_doc)){
    //     if (kdl_parser::treeFromUrdfModel(droneModel, droneModelTree)){
    //         ROS_INFO("Model loaded!");
    //         rsPub = robot_state_publisher::RobotStatePublisher(droneModelTree, droneModel);
    //     } else {
    //         ROS_ERROR("Failed to construct kdl tree");
    //     }
    // } else {
    //     ROS_ERROR("Failed to parse urdf robot model");
    // }
    ros::Rate loop_rate(30);
    // ros::NodeHandle n;
    
    
    joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    

    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    
    const double degree = M_PI/180;
    double tilt = 0, tinc = degree, swivel=0, angle=0, height=0, hinc=0.005;
    sub_ = n.subscribe("/Drone/IMU_KAL", 1, &DroneViz::IMUCallback, this);

    // ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
}

void DroneViz::IMUCallback(const sensor_msgs::Imu imuData){
        joint_state.header.stamp = ros::Time::now();
        odom_trans.header.stamp = ros::Time::now();
        // odom_trans.transform.translation.x = cos(angle)*2;
        // odom_trans.transform.translation.y = sin(angle)*2;
        // odom_trans.transform.translation.z = .7;
        odom_trans.transform.rotation = 
        tf::createQuaternionMsgFromRollPitchYaw(
            -imuData.linear_acceleration.y*DEGREE2RAD, 
            imuData.linear_acceleration.x*DEGREE2RAD, 
            imuData.linear_acceleration.z*DEGREE2RAD);

        //send the joint state and transform
        broadcaster.sendTransform(odom_trans);
        joint_pub.publish(joint_state);
        // Create new robot state
        // tilt += tinc;
        // if (tilt<-.5 || tilt>0) tinc *= -1;
        // height += hinc;
        // if (height>.2 || height<0) hinc *= -1;
        // swivel += degree;
        // angle += degree/4;
}

DroneViz::~DroneViz(){

}