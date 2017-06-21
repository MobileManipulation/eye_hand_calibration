#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <eyehand_calib/Capture.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <aruco_msgs/MarkerArray.h>
#include <geometry_msgs/TransformStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2_ros/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>

#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <atomic>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>


class EyehandObserver
{
public:
  void data_callback(const sensor_msgs::JointState::ConstPtr& states,
                     const sensor_msgs::Image::ConstPtr& image,
                     const sensor_msgs::PointCloud2::ConstPtr& points,
                     const aruco_msgs::MarkerArray::ConstPtr& tags)
  {
    if (!capture_active) return;
    std::cout << "data_callback: " << current_frame+1 << '/' << frame_count << std::endl;

    // Put data in buffer
    buffer[current_frame].states = states;
    buffer[current_frame].image = image;
    buffer[current_frame].points = points;
    buffer[current_frame].tags = tags;

    current_frame++;
    capture_active = (current_frame < frame_count);
  }

  bool init_capture(eyehand_calib::CaptureRequest& req, eyehand_calib::CaptureResponse& res)
  {
    std::cout << "Starting capture: " << req.frame_count << std::endl;
    if (capture_active) return false;
    if (req.frame_count == 0) return false;

    // Prepare to save data
    buffer.resize(req.frame_count);

    frame_count = req.frame_count;
    current_frame = 0;
    capture_active = true;

    // Wait for data to queue up
    // Main thread will set capture_active to false when it's ready
    auto start = std::chrono::steady_clock::now();
    auto timeout = std::chrono::seconds(10);
    while (ros::ok() and capture_active)
    {
      // Check for timeout
      auto dur = std::chrono::steady_clock::now() - start;
      if (dur > timeout)
      {
        capture_active = false;
        std::cout << "Capture timeout! Aborting!" << std::endl;
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "Capture complete!" << std::endl;

    std::cout << "Compute transforms... ";
    for (auto& frame : buffer)
    {
      // Grab the required transforms
      frame.ee_xform = tfBuffer.lookupTransform("calibration_target", "world", frame.states->header.stamp);
      frame.ptu_xform = tfBuffer.lookupTransform("kinect1_rgb_optical_frame", "world", frame.states->header.stamp);
    }
    std::cout << "Done!" << std::endl;

    // Save data
    std::cout << "Serializing data..." << std::endl;
    serialize_data(req);
    std::cout << "Datapoint saved!" << std::endl;

    res.captured_frames = req.frame_count;
    return true;
  }

  std::string create_filename(const std::string& dir, uint16_t data_point,
                              uint16_t frame_id, const std::string& suffix)
  {
      std::ostringstream filename;
      // Formatting
      filename << std::setfill('0') << std::right;
      filename << dir;
      filename << '/' << std::setw(3) << data_point;
      filename << '_' << std::setw(2) << frame_id;
      filename << '_' << suffix;
      return filename.str();
  }

  template<class T>
  std::ostream& serialize_pose(std::ostream& os, const T& pose)
  {
      os << pose.position.x << ',';
      os << pose.position.y << ',';
      os << pose.position.z << ',';
      os << pose.orientation.x << ',';
      os << pose.orientation.y << ',';
      os << pose.orientation.z << ',';
      os << pose.orientation.w;
      return os;
  }

  template<class T>
  std::ostream& serialize_xform(std::ostream& os, const T& xform)
  {
      os << xform.translation.x << ',';
      os << xform.translation.y << ',';
      os << xform.translation.z << ',';
      os << xform.rotation.x << ',';
      os << xform.rotation.y << ',';
      os << xform.rotation.z << ',';
      os << xform.rotation.w;
      return os;
  }

  void serialize_data(const eyehand_calib::CaptureRequest& req)
  {
    // Format datapoint number like this: 042
    std::ostringstream dp;
    dp << std::setw(3) << std::setfill('0') << std::right << req.data_point;

    // Build up relevant filenames
    std::string dp_csv = req.data_path + '/' + dp.str() + "_raw_data.csv";
    std::string dp_dir = req.data_path + '/' + dp.str();

    // Validate filenames...
    std::cout << "dp_csv: " << dp_csv << std::endl;
    std::cout << "dp_dir: " << dp_dir << std::endl;

    // Create folders
    boost::filesystem::create_directory(req.data_path);
    boost::filesystem::create_directory(dp_dir);

    // Open main CSV file
    std::ofstream csv_stream(dp_csv);

    // Loop over frames
    for (uint16_t frame_id = 0; frame_id < buffer.size(); frame_id++)
    {
      // Create frame-specific filenames
      std::string pcd_file = create_filename(dp_dir, req.data_point, frame_id, "raw_points.pcd");
      std::string img_file = create_filename(dp_dir, req.data_point, frame_id, "image_rect_color.png");

      std::cout << pcd_file << std::endl;
      std::cout << img_file << std::endl;

      auto& frame = buffer[frame_id];

      // Frame number
      csv_stream << std::noshowpos;
      csv_stream << frame_id << ",\t";

      // Timestamp
      auto&& stamp = frame.states->header.stamp.toBoost();
      csv_stream << boost::posix_time::to_iso_string(stamp) << ",\t";

      // Set format for remaining numbers -- all floats
      csv_stream << std::setprecision(6) << std::scientific << std::showpos;

      ////////////////////////////////////
      // Items that don't vary per frame
      ////////////////////////////////////
      // Calibration plate desired location in camera space
      serialize_pose(csv_stream, req.camera_target.pose) << ",\t";
      // Calibration plate desired location in camera space
      serialize_pose(csv_stream, req.world_target.pose) << ",\t";
      // Joint states: Arm & PTU conf, desired
      csv_stream << req.conf.robotHead[0] << ','; // pan
      csv_stream << req.conf.robotHead[1] << ','; // tilt
      for (auto j = 0; j < 7; j++)
        csv_stream << req.conf.robotRightArm[j] << ','; // iiwa_link_0-7

      csv_stream << "\t\t";

      ////////////////////////////////////
      // Items that do vary per frame
      ////////////////////////////////////
      // AR Tag location (pose measured by camera)
      auto& tag = frame.tags->markers[0].pose.pose;
      serialize_pose(csv_stream, tag) << ",\t";
      // Calibration plate location relative to world (pose measured by Kuka)
      serialize_xform(csv_stream, frame.ee_xform.transform) << ",\t";

      // Joint states: Arm & PTU conf, measured by Kuka
      auto& states = frame.states->position;
      for (auto j = 3; j < 12; j++)
        csv_stream << states[j] << ',';
      csv_stream << '\t';

      // kinect1_rgb_optical_frame location relative to world (camera location, measured)
      serialize_xform(csv_stream, frame.ptu_xform.transform) << std::endl;


      // Save the pointcloud
      pcl::PointCloud<pcl::PointXYZRGB> cloud;
      pcl::fromROSMsg(*(frame.points), cloud);
      pcl::io::savePCDFileBinary(pcd_file, cloud);

      // Save the image
      cv_bridge::CvImagePtr bridge = cv_bridge::toCvCopy(frame.image, sensor_msgs::image_encodings::BGR8);
      cv::imwrite(img_file, bridge->image);
    }

    // Close main CSV
    csv_stream.close();
  }

  EyehandObserver()
  : capture_active(false),
    tfListener(tfBuffer)
  {
    // Listen for ALL THE DATA!
    joint_states_sub.subscribe(nh, "/joint_states", 10);
    raw_image_sub.subscribe(nh, "/kinect1/rgb/image_rect_color", 10);
    pointcloud_sub.subscribe(nh, "/kinect1/depth_registered/points", 10);
    aruco_sub.subscribe(nh, "/aruco_tags/markers", 10);
    channel.reset(new SyncChannel(SyncPolicy(10), joint_states_sub, raw_image_sub, pointcloud_sub, aruco_sub));
    channel->registerCallback(&EyehandObserver::data_callback, this);

    // Spin service call in its own thread so it doesn't block
    service_nh.setCallbackQueue(&service_queue);
    capture_server = service_nh.advertiseService("capture_data", &EyehandObserver::init_capture, this);
    server_thread = std::thread([this](){
      while(ros::ok()){
        service_queue.callAvailable(ros::WallDuration());
      }
      std::cout << "Thread complete" << std::endl;
    });
  }

  ~EyehandObserver()
  {
    server_thread.join();
  }
private:
  // Data listeners
  ros::NodeHandle nh;
  message_filters::Subscriber<sensor_msgs::JointState> joint_states_sub;
  message_filters::Subscriber<sensor_msgs::Image> raw_image_sub;
  message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub;
  message_filters::Subscriber<aruco_msgs::MarkerArray> aruco_sub;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::JointState,
                                                          sensor_msgs::Image,
                                                          sensor_msgs::PointCloud2,
                                                          aruco_msgs::MarkerArray> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> SyncChannel;
  std::shared_ptr<SyncChannel> channel;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;


  // Service handler
  ros::NodeHandle service_nh;
  ros::CallbackQueue service_queue;
  ros::ServiceServer capture_server;
  std::thread server_thread;

  std::atomic<bool> capture_active;
  std::atomic<uint16_t> current_frame;
  std::atomic<uint16_t> frame_count;


  // Data management
  struct BufferElement {
      sensor_msgs::JointState::ConstPtr states;
      sensor_msgs::Image::ConstPtr image;
      sensor_msgs::PointCloud2::ConstPtr points;
      aruco_msgs::MarkerArray::ConstPtr tags;
      geometry_msgs::TransformStamped ee_xform;
      geometry_msgs::TransformStamped ptu_xform;
  };
  typedef std::vector<BufferElement> DataBuffer;
  DataBuffer buffer;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "eyehand_observer");
  EyehandObserver obs;
  std::cout << "Commencing spin" << std::endl;
  ros::spin();
  std::cout << "Spun." << std::endl;
}
