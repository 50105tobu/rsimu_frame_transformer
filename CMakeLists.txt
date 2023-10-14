cmake_minimum_required(VERSION 3.0.2) #iru?
project(rsimu_frame_transformer)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  tf2_ros
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu_transformer_node src/imu_transformer_node.cpp)

target_link_libraries(imu_transformer_node
  ${catkin_LIBRARIES}
)
