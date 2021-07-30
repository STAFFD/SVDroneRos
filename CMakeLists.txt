cmake_minimum_required(VERSION 3.0.2)
project(SVDroneMon)

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs 
    genmsg
    geometry_msgs
    sensor_msgs
    serial
    tf
)

generate_messages(
    DEPENDENCIES
    geometry_msgs 
    std_msgs 
    sensor_msgs)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_drone_mon_node src/drone_mon.cpp src/drone_mon_node.cpp)
target_link_libraries(${PROJECT_NAME}_drone_mon_node ${catkin_LIBRARIES} ${rosserial_LIBRARIES})
add_dependencies(${PROJECT_NAME}_drone_mon_node ${PROJECT_NAME}_generate_messages_cpp)

add_executable(${PROJECT_NAME}_drone_viz_node src/drone_viz.cpp src/drone_viz_node.cpp)
target_link_libraries(${PROJECT_NAME}_drone_viz_node ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_drone_viz_node ${PROJECT_NAME}_generate_messages_cpp)


