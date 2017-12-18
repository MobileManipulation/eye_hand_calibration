#include "correct_realsense_frame.h"

CorrectRealSenseFrame::CorrectRealSenseFrame(): have_kinect_aruco_pose_(false), have_realsense_aruco_pose_(false)
{
  // Get params
  get_params();

  // Setup the subscribers and publishers
  setup_subs_pubs_srvs();

}

// Get ROS params
void CorrectRealSenseFrame::get_params()
{
  nh_.getParam("world_frame", world_frame_);
  nh_.getParam("kinect_frame", kinect_frame_);
  nh_.getParam("realsense_frame", realsense_frame_);
  nh_.getParam("kinect_aruco_topic", kinect_aruco_topic_);
  nh_.getParam("realsense_aruco_topic", realsense_aruco_topic_);
  nh_.getParam("compute_delta_srv_name", compute_delta_srv_name_);
}

// Setup ROS subscribers, publishers, and services
void CorrectRealSenseFrame::setup_subs_pubs_srvs()
{
  // Initializing the tf2 listener
  tf2_listener_ = std::unique_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(tf2_buffer_));

  kinect_aruco_sub_ = nh_.subscribe(kinect_aruco_topic_, 1, &CorrectRealSenseFrame::kinect_aruco_cb, this);
  realsense_aruco_sub_ = nh_.subscribe(realsense_aruco_topic_, 1, &CorrectRealSenseFrame::realsense_aruco_cb, this);

  compute_delta_srv_ = nh_.advertiseService(compute_delta_srv_name_, &CorrectRealSenseFrame::compute_delta_srv, this);
}

// Lookup tfs
void CorrectRealSenseFrame::lookup_tfs()
{
  w_T_kinect_tf_ = tf2_buffer_.lookupTransform(world_frame_, kinect_frame_, ros::Time(0), ros::Duration(3.0));
  w_T_kinect_ = transform_stamped_2_eigen(w_T_kinect_tf_);
  std::cout << "w_T_kinect_: " << std::endl << w_T_kinect_ << std::endl;

  w_T_realsense_tf_ = tf2_buffer_.lookupTransform(world_frame_, realsense_frame_, ros::Time(0), ros::Duration(3.0));
  w_T_realsense_ = transform_stamped_2_eigen(w_T_realsense_tf_);
  std::cout << "w_T_realsense_: " << std::endl << w_T_realsense_ << std::endl;
}

// Handle Kinect Aruco detections
void CorrectRealSenseFrame::kinect_aruco_cb(const aruco_msgs::MarkerArrayConstPtr& msg)
{
  std::vector<aruco_msgs::Marker> markers = msg->markers;

  // Make sure that there is a detection
  if (!markers.empty())
  {
    aruco_msgs::Marker aruco_detection = markers[0];
    kinect_T_aruco_ = pose_msg_2_transform(aruco_detection.pose.pose);

    if (!have_kinect_aruco_pose_)
    {
      std::cout << "kinect_T_aruco_: " << std::endl << kinect_T_aruco_ << std::endl;

      have_kinect_aruco_pose_ = true;
    }
  }
}

// Handle RealSense Aruco detections
void CorrectRealSenseFrame::realsense_aruco_cb(const aruco_msgs::MarkerArrayConstPtr& msg)
{
  std::vector<aruco_msgs::Marker> markers = msg->markers;

  // Make sure that there is a detection
  if (!markers.empty())
  {
    aruco_msgs::Marker aruco_detection = markers[0];
    realsense_T_aruco_ = pose_msg_2_transform(aruco_detection.pose.pose);

    if (!have_realsense_aruco_pose_)
    {
      std::cout << "realsense_T_aruco_: " << std::endl << realsense_T_aruco_ << std::endl;

      have_realsense_aruco_pose_ = true;
    }
  }
}

// Computing the delta to put the RealSense detection in line with the Kinect detection
void CorrectRealSenseFrame::compute_delta()
{
  Eigen::Matrix4d w_T_kinect_aruco_ = w_T_kinect_ * kinect_T_aruco_;
  // Kinect detection in the RealSense frame
  Eigen::Matrix4d realsense_T_kinect_aruco = w_T_realsense_.inverse() * w_T_kinect_aruco_;

  // Computing the delta to put the RealSense detection in line with the Kinect detection
  Eigen::Matrix4d realsense_aruco_T_kinect_aruco = realsense_T_aruco_.inverse() * realsense_T_kinect_aruco;
  std::cout << "realsense_aruco_T_kinect_aruco: " << std::endl << realsense_aruco_T_kinect_aruco << std::endl;

  double roll, pitch, yaw;
  Eigen::Matrix3d rot = realsense_aruco_T_kinect_aruco.topLeftCorner(3,3);
  Eigen::Quaterniond quat(rot);
  tf::Quaternion quat_tf;
  tf::quaternionEigenToTF(quat, quat_tf);
  tf::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);

  std::cout << "Roll: " << roll << " Pitch: " << pitch << " Yaw: " << yaw << std::endl;
}

// Compute the delta between the Kinect and RealSense Aruco detections
bool CorrectRealSenseFrame::compute_delta_srv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
  if (have_realsense_aruco_pose_ && have_kinect_aruco_pose_)
  {  
    lookup_tfs();
    compute_delta();
  }

  else
  {
    ROS_FATAL("DON'T HAVE REALSENSE AND KINECT ARUCO POSES!");
  }
}

// Converts a ROS Geometry Pose msg into an Eigen transform
Eigen::Matrix4d CorrectRealSenseFrame::pose_msg_2_transform(const geometry_msgs::Pose& pose_msg)
{
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

  // Convert pose message to transform
  geometry_msgs::Point position = pose_msg.position;
  geometry_msgs::Quaternion quat = pose_msg.orientation;

  tf::Quaternion tf_quat;
  tf::quaternionMsgToTF(quat, tf_quat);
  Eigen::Quaterniond eigen_quat;
  tf::quaternionTFToEigen(tf_quat, eigen_quat);
  transform.topLeftCorner(3,3) = eigen_quat.toRotationMatrix();

  Eigen::Vector3d trans;
  trans(0) = position.x;
  trans(1) = position.y;
  trans(2) = position.z;
  transform.topRightCorner(3,1) = trans;

  return transform;
}

// Converts a tf::Transform to an Eigen 4x4 Transformation matrix
Eigen::Matrix4d CorrectRealSenseFrame::transform_stamped_2_eigen(const geometry_msgs::TransformStamped& transform_stamped)
{
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

  Eigen::Vector3d trans;
  trans(0) = transform_stamped.transform.translation.x;
  trans(1) = transform_stamped.transform.translation.y;
  trans(2) = transform_stamped.transform.translation.z;
  transform.topRightCorner(3,1) = trans;

  Eigen::Quaterniond quat(transform_stamped.transform.rotation.w, transform_stamped.transform.rotation.x, transform_stamped.transform.rotation.y, transform_stamped.transform.rotation.z);
  transform.topLeftCorner(3,3) = quat.toRotationMatrix();

  return transform;
}

// Main Loop
int main(int argc, char** argv)
{
  ros::init(argc, argv, "correct_realsense_frame_node");

  CorrectRealSenseFrame correct_realsense_frame_node;
  ros::spin();

  return 0;
}