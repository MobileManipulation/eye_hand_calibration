cmake_minimum_required(VERSION 2.8.3)
project(eye_hand_calibration)

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
  segmentation
)
find_package(OpenCV REQUIRED)

add_service_files(FILES
  Capture.srv
)

generate_messages(DEPENDENCIES std_msgs robot_msgs geometry_msgs)

catkin_package(
  #INCLUDE_DIRS include
 # LIBRARIES
)

macro(calib_target target_name)
  add_executable(${target_name} src/${target_name}.cpp)
  target_include_directories(${target_name}
    PUBLIC
      ${catkin_INCLUDE_DIRS}
      ${Boost_INCLUDE_DIRS}
      ${PCL_INCLUDE_DIRS}
      ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(${target_name}
    PUBLIC
      ${catkin_LIBRARIES}
      ${Boost_LIBRARIES}
      ${PCL_LIBRARIES}
      ${OpenCV_LIBRARIES}
  )
  target_compile_features(${target_name}
    PRIVATE
      cxx_auto_type
      cxx_lambdas
      cxx_nonstatic_member_init
      cxx_range_for
  )
endmacro(calib_target)

calib_target(observation_node)
calib_target(crop_pointcloud)
calib_target(csv_merge1)
calib_target(plane_fit)
