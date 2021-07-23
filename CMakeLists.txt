cmake_minimum_required(VERSION 3.0.2)
project(SVDroneMonitor)

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs 
    genmsg
    sensor_msgs
    serial
)



generate_messages(DEPENDENCIES std_msgs sensor_msgs)
catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/drone_monitor.cpp src/drone_monitor_node.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${rosserial_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)


