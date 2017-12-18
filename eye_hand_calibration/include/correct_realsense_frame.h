#ifndef CORRECT_REALSENSE_FRAME_H
#define CORRECT_REALSENSE_FRAME_H

// C++ Stuff
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <unordered_map>

// For ROS
#include <ros/ros.h> 

// For tf2
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
// For some conversion fxns
#include <tf/transform_datatypes.h>
#include "tf_conversions/tf_eigen.h"

// For ROS msgs
#include <std_msgs/Header.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <aruco_msgs/MarkerArray.h>
#include <aruco_msgs/Marker.h>

class CorrectRealSenseFrame {
  public:
    CorrectRealSenseFrame();

  private:
    // ROS Stuff
    ros::NodeHandle nh_;
    // tf2 stuff
    tf2_ros::Buffer tf2_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

    bool have_kinect_aruco_pose_;
    bool have_realsense_aruco_pose_;

    std::string world_frame_;
    std::string kinect_frame_;
    std::string realsense_frame_;
    std::string kinect_aruco_topic_;
    std::string realsense_aruco_topic_;
    std::string compute_delta_srv_name_;

    geometry_msgs::TransformStamped w_T_kinect_tf_;
    geometry_msgs::TransformStamped w_T_realsense_tf_;

    Eigen::Matrix4d w_T_kinect_;
    Eigen::Matrix4d w_T_realsense_;    
    Eigen::Matrix4d kinect_T_aruco_;
    Eigen::Matrix4d realsense_T_aruco_;

    ros::Subscriber kinect_aruco_sub_;
    // Handle Kinect Aruco detections
    void kinect_aruco_cb(const aruco_msgs::MarkerArrayConstPtr& msg);

    ros::Subscriber realsense_aruco_sub_;
    // Handle RealSense Aruco detections
    void realsense_aruco_cb(const aruco_msgs::MarkerArrayConstPtr& msg);

    ros::ServiceServer compute_delta_srv_;
    // Compute the delta between the Kinect and RealSense Aruco detections
    bool compute_delta_srv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

    // Get ROS params
    void get_params();

    // Setup ROS subscribers, publishers, and services
    void setup_subs_pubs_srvs();

    // Lookup tfs
    void lookup_tfs();

    // Computing the delta to put the RealSense detection in line with the Kinect detection
    void compute_delta();

    // Converts a ROS Geometry Pose msg into an Eigen transform
    Eigen::Matrix4d pose_msg_2_transform(const geometry_msgs::Pose& pose_msg);
    // Converts a tf::Transform to an Eigen 4x4 Transformation matrix
    Eigen::Matrix4d transform_stamped_2_eigen(const geometry_msgs::TransformStamped& transform_stamped);
};

#endif // CORRECT_REALSENSE_FRAME_H