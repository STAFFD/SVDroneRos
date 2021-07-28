#include "drone_monitor.h"

DroneMonitor::DroneMonitor(/* args */){

    pub_ = nh.advertise<sensor_msgs::Imu>("/Drone/IMU", 1);
    pub_kal = nh.advertise<sensor_msgs::Imu>("/Drone/IMU_KAL", 1);
    try{
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(57600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e){
        ROS_ERROR_STREAM("Unable to open port ");
    }
    sensor_msgs::Imu imu;
    sensor_msgs::Imu imu_kal;
    while(ros::ok()){
        if(ser.available() >= TRANS_TOTAL){
            if (ser.read() == "S"){
                if(ser.read() == "V"){
                    ser.read(resultBuffer, TRANS_TOTAL);

                    droneData.angle_roll = (float)((int16_t)(resultBuffer[TRANS_ANG_ROLL_HIGH] << 8 | resultBuffer[TRANS_GYRO_ROLL_LOW]))/100;
                    droneData.angle_pitch = (float)((int16_t)(resultBuffer[TRANS_ANG_PITCH_HIGH] << 8 | resultBuffer[TRANS_ANG_PITCH_LOW]))/100;
                    droneData.angle_yaw = (float)((int16_t)(resultBuffer[TRANS_ANG_YAW_HIGH] << 8 | resultBuffer[TRANS_ANG_YAW_LOW]))/100;
                    droneData.gyro_roll = (float)((int16_t)(resultBuffer[TRANS_GYRO_ROLL_HIGH] << 8 | resultBuffer[TRANS_GYRO_ROLL_LOW]))/100;
                    droneData.gyro_pitch = (float)((int16_t)(resultBuffer[TRANS_GYRO_PITCH_HIGH] << 8 | resultBuffer[TRANS_GYRO_PITCH_LOW]))/100;
                    droneData.gyro_yaw = (float)((int16_t)(resultBuffer[TRANS_GYRO_YAW_HIGH] << 8 | resultBuffer[TRANS_GYRO_YAW_LOW]))/100;
                    
                    droneData.kal_roll = (float)((int16_t)(resultBuffer[TRANS_KAL_ROLL_HIGH] << 8 | resultBuffer[TRANS_KAL_ROLL_LOW]))/100;
                    droneData.kal_pitch = (float)((int16_t)(resultBuffer[TRANS_KAL_PITCH_HIGH] << 8 | resultBuffer[TRANS_KAL_PITCH_LOW]))/100;

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

                    pub_.publish(imu);
                    pub_kal.publish(imu_kal);
                    #ifdef DEBUG_PRINT
                        cout << "angle_roll: " << droneData.angle_roll << endl;
                        cout << "angle_pitch: " << droneData.angle_pitch << endl;
                        cout << "angle_yaw: " << droneData.angle_yaw << endl;
                        cout << "gyro_roll: " << droneData.gyro_roll << endl;
                        cout << "gyro_pitch: " << droneData.gyro_pitch << endl;
                        cout << "gyro_yaw: " << droneData.gyro_yaw << endl;
                    #endif
                }
            }
        }
    }
}

DroneMonitor::~DroneMonitor()
{
}