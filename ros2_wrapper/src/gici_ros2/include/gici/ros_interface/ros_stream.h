/**
* @Function: Handle ROS 2 stream publishing and subscribing
*
* Original author: Cheng Chi <chichengcn@sjtu.edu.cn>
* ROS 2 (rclcpp/ament) port for the gici_research_standard UrbanNav setup.
**/
#pragma once

#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <functional>
#include <glog/logging.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <gici_ros2_msgs/msg/glonass_ephemeris.hpp>
#include <gici_ros2_msgs/msg/gnss_antenna_position.hpp>
#include <gici_ros2_msgs/msg/gnss_ephemerides.hpp>
#include <gici_ros2_msgs/msg/gnss_ionosphere_parameter.hpp>
#include <gici_ros2_msgs/msg/gnss_observations.hpp>
#include <gici_ros2_msgs/msg/gnss_ssr_code_biases.hpp>
#include <gici_ros2_msgs/msg/gnss_ssr_phase_biases.hpp>
#include <gici_ros2_msgs/msg/gnss_ssr_ephemerides.hpp>

#include "gici/stream/node_handle.h"
#include "gici/estimate/estimating.h"
#include "gici/ros_interface/ros_publisher.h"

namespace gici {

// ROS data format
enum class RosDataFormat {
  Image,
  Imu,
  GnssRaw,
  PoseStamped,
  PoseWithCovarianceStamped,
  Odometry,
  NavSatFix,
  Marker,
  Path
};

// GNSS raw data format
enum class RosGnssDataFormat {
  Observations,
  Ephemerides,
  AntennaPosition,
  IonosphereParameter,
  CodeBias,
  PhaseBias,
  EphemeridesCorrection
};

class RosStream : public Streaming {
public:
  using DataCallback = Streaming::DataCallback;
  using PipelineCallback = Streaming::PipelineConvert;

  RosStream(rclcpp::Node::SharedPtr node, const NodeOptionHandlePtr& nodes, int istreamer);
  ~RosStream();

  // Check if valid
  inline bool valid() { return valid_; }

  // Set estimator data callback
  void setDataCallback(const DataCallback& callback) override {
    data_callbacks_.push_back(callback);
  }

  // Output data callback
  void outputDataCallback(
    const std::string tag, const std::shared_ptr<DataCluster>& data) override;

  // Pipeline sends input data to logging stream
  void pipelineCallback(const std::string& tag, const std::shared_ptr<DataCluster>& data);

  // Get tag
  std::string getTag() { return tag_; }

  // Get I/O type
  StreamIOType getIoType() { return io_type_; }

  // Bind input and logging streams (ROS to ROS)
  static void bindLogWithInput();

  // Get instantiated objects
  static std::vector<RosStream *>& getObjects() { return static_this_; }

private:
  // Send solution data to ROS topic
  void solutionOutputCallback(
    std::string tag, Solution& solution);

  // Send featured image to ROS topic
  void featuredImageOutputCallback(FramePtr& frame);

  // Send features as marker to ROS topic
  void mapPointOutputCallback(MapPtr& map);

  // Send GNSS raw data to ROS topics
  void gnssRawDataOutputCallback(DataCluster::GNSS& gnss);

  // Send IMU data to ROS topic
  void imuDataOutputCallback(DataCluster::IMU& imu);

  // Send image data to ROS topic
  void imageDataOutputCallback(DataCluster::Image& image);

  // ROS callbacks
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void gnssObservationsCallback(const gici_ros2_msgs::msg::GnssObservations::ConstSharedPtr msg);
  void gnssEphemeridesCallback(const gici_ros2_msgs::msg::GnssEphemerides::ConstSharedPtr msg);
  void gnssAntennaPositionCallback(const gici_ros2_msgs::msg::GnssAntennaPosition::ConstSharedPtr msg);
  void gnssIonosphereParameterCallback(const gici_ros2_msgs::msg::GnssIonosphereParameter::ConstSharedPtr msg);
  void gnssSsrCodeBiasesCallback(const gici_ros2_msgs::msg::GnssSsrCodeBiases::ConstSharedPtr msg);
  void gnssSsrPhaseBiasesCallback(const gici_ros2_msgs::msg::GnssSsrPhaseBiases::ConstSharedPtr msg);
  void gnssSsrEphemeridesCallback(const gici_ros2_msgs::msg::GnssSsrEphemerides::ConstSharedPtr msg);
  void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
  void navSatFixCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);

protected:
  // Stream control
  std::string tag_;
  bool valid_;
  std::string input_ros_stream_tag_;  // for ROS to ROS pipeline
  StreamIOType io_type_;
  std::vector<DataCallback> data_callbacks_;  // call external function to send data out
  using PipelinesRosToRos = std::vector<PipelineCallback>;
  PipelinesRosToRos pipeline_ros_to_ros_; // sending data from ROS input to ROS log

  // ROS handles
  rclcpp::Node::SharedPtr node_;
  std::vector<rclcpp::PublisherBase::SharedPtr> publishers_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscribers_;
  std::vector<RosGnssDataFormat> gnss_formats_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tranform_broadcaster_;
  GeoCoordinatePtr input_coordinate_;
  std::string frame_id_;
  std::string subframe_id_;
  std::string topic_name_;
  double marker_scale_ = 0.1;
  RosDataFormat data_format_;
  int queue_size_;
  std::unique_ptr<PathPublisher> path_publisher_;

  // Static variables for stream binding
  static std::vector<RosStream *> static_this_;
};

}
