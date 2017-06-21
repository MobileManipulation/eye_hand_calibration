cmake_minimum_required(VERSION 2.8.3)
project(eyehand_calib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  sensor_msgs
  geometry_msgs
  robot_msgs
  trajectory_msgs
  cv_bridge
)
find_package(Boost REQUIRED COMPONENTS
  filesystem
)
find_package(PCL 1.8 REQUIRED COMPONENTS
  common
  io
)
find_package(OpenCV REQUIRED)

add_service_files(FILES
  Capture.srv
)

generate_messages(DEPENDENCIES std_msgs robot_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
 # LIBRARIES
)


add_executable(observation_node src/observation_node.cpp)
target_include_directories(observation_node
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(observation_node
  PUBLIC
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
target_compile_features(observation_node
  PRIVATE
    cxx_auto_type
    cxx_lambdas
    cxx_nonstatic_member_init
    cxx_range_for
)
