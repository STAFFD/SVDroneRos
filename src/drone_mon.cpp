#include "drone_mon.h"

DroneMonitor::DroneMonitor(/* args */){
    pub_ = nh.advertise<sensor_msgs::Imu>("/Drone/IMU", 1);
    pub_kal = nh.advertise<sensor_msgs::Imu>("/Drone/IMU_KAL", 1);
    string usbPort;
    ros::param::get(ros::this_node::getName()+"/usb_port", usbPort);
    int baudrate;
    ros::param::get(ros::this_node::getName()+"/baudrate", baudrate);
    ros::Rate loop_rate(1);
    while(true){
        try{
            cout << "Opening usb port: "<< usbPort << "=>";
            ser.setPort(usbPort);
            ser.setBaudrate(baudrate);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser.setTimeout(to);
            ser.open();
            cout << "Done!" << endl;
            break;
        } catch (serial::IOException& e){
            ROS_ERROR_STREAM("Unable to open port! retry...");
            loop_rate.sleep();
        }
    }

}

void DroneMonitor::run(){
    while(ros::ok()){
        if (ser.available() >= TRANS_TOTAL){
            if(ser.read() == "S" && ser.read() == "V"){
                ser.read(resultBuffer, TRANS_TOTAL);
                droneData.angle_roll = (float)((int16_t)(resultBuffer[TRANS_ANG_ROLL_HIGH] << 8 | resultBuffer[TRANS_GYRO_ROLL_LOW]))/100;
                droneData.angle_pitch = (float)((int16_t)(resultBuffer[TRANS_ANG_PITCH_HIGH] << 8 | resultBuffer[TRANS_ANG_PITCH_LOW]))/100;
                droneData.angle_yaw = (float)((int16_t)(resultBuffer[TRANS_ANG_YAW_HIGH] << 8 | resultBuffer[TRANS_ANG_YAW_LOW]))/100;
                droneData.gyro_roll = (float)((int16_t)(resultBuffer[TRANS_GYRO_ROLL_HIGH] << 8 | resultBuffer[TRANS_GYRO_ROLL_LOW]))/100;
                droneData.gyro_pitch = (float)((int16_t)(resultBuffer[TRANS_GYRO_PITCH_HIGH] << 8 | resultBuffer[TRANS_GYRO_PITCH_LOW]))/100;
                droneData.gyro_yaw = (float)((int16_t)(resultBuffer[TRANS_GYRO_YAW_HIGH] << 8 | resultBuffer[TRANS_GYRO_YAW_LOW]))/100;
                
                droneData.kal_roll = (float)((int16_t)(resultBuffer[TRANS_KAL_ROLL_HIGH] << 8 | resultBuffer[TRANS_KAL_ROLL_LOW]))/100;
                droneData.kal_pitch = (float)((int16_t)(resultBuffer[TRANS_KAL_PITCH_HIGH] << 8 | resultBuffer[TRANS_KAL_PITCH_LOW]))/100;


                droneData.q1 = (float)((int16_t)(resultBuffer[TRANS_Q1_HIGH] << 8 | resultBuffer[TRANS_Q1_LOW]))/10000;
                droneData.q2 = (float)((int16_t)(resultBuffer[TRANS_Q2_HIGH] << 8 | resultBuffer[TRANS_Q2_LOW]))/10000;
                droneData.q3 = (float)((int16_t)(resultBuffer[TRANS_Q3_HIGH] << 8 | resultBuffer[TRANS_Q3_LOW]))/10000;
                droneData.q4 = (float)((int16_t)(resultBuffer[TRANS_Q4_HIGH] << 8 | resultBuffer[TRANS_Q4_LOW]))/10000;


                imu.header.stamp = ros::Time::now();
                imu.angular_velocity.x = droneData.gyro_roll;
                imu.angular_velocity.y = droneData.gyro_pitch;
                imu.angular_velocity.z = droneData.gyro_yaw;
                imu.linear_acceleration.x = droneData.angle_roll;
                imu.linear_acceleration.y = droneData.angle_pitch;
                imu.linear_acceleration.z = droneData.angle_yaw;



                imu_kal.header.stamp = ros::Time::now();
                imu_kal.linear_acceleration.x = droneData.kal_roll;
                imu_kal.linear_acceleration.y = droneData.kal_pitch;

                imu_kal.orientation.x = droneData.q2;
                imu_kal.orientation.y = droneData.q3;
                imu_kal.orientation.z = droneData.q4;
                imu_kal.orientation.w = droneData.q1;
                pub_.publish(imu);
                pub_kal.publish(imu_kal);
                #ifdef DEBUG_PRINT
                    cout << "angle_roll: " << droneData.angle_roll << endl;
                    cout << "angle_pitch: " << droneData.angle_pitch << endl;
                    cout << "angle_yaw: " << droneData.angle_yaw << endl;
                    cout << "gyro_roll: " << droneData.gyro_roll << endl;
                    cout << "gyro_pitch: " << droneData.gyro_pitch << endl;
                    cout << "gyro_yaw: " << droneData.gyro_yaw << endl;
                    
                    cout << "q1: " << droneData.q1 << endl;
                    cout << "q2: " << droneData.q2 << endl;
                    cout << "q3: " << droneData.q3 << endl;
                    cout << "q4: " << droneData.q4 << endl;
                #endif
            }

        }
    }
}

DroneMonitor::~DroneMonitor(){
    ser.close();
}