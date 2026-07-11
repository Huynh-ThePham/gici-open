/**
* @Function: ROS 2 publishers (port of chichengcn/gici-open ROS1 wrapper)
*
* Original author: Cheng Chi <chichengcn@sjtu.edu.cn>
* ROS 2 (rclcpp/ament) port for the gici_research_standard UrbanNav setup.
**/
#pragma once

#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "gici/utility/svo.h"
#include "gici/stream/formator.h"

namespace gici {

// Convert double seconds (GICI internal time) to rclcpp::Time
inline rclcpp::Time toRosTime(double t) {
  if (t < 0.0) t = 0.0;
  return rclcpp::Time(static_cast<int64_t>(t * 1.0e9), RCL_ROS_TIME);
}

// Publish raw image
void publishImage(const rclcpp::PublisherBase::SharedPtr& pub,
  const cv::Mat& image, const rclcpp::Time time);
void publishImage(const rclcpp::PublisherBase::SharedPtr& pub,
  const FramePtr& frame, const rclcpp::Time time, const std::string& encoding);

// Publish image with features
void publishFeaturedImage(const rclcpp::PublisherBase::SharedPtr& pub,
  const FramePtr& frame, const rclcpp::Time time);

// Publish landmarks
void publishLandmarks(const rclcpp::PublisherBase::SharedPtr& pub,
  const MapPtr& map, const rclcpp::Time time,
  std::string frame_id, double marker_scale = 0.1);

// Publish pose
void publishPoseStamped(const rclcpp::PublisherBase::SharedPtr& pub,
  const Transformation& pose, const rclcpp::Time time,
  std::string frame_id);

// Publish pose with covariance
void publishPoseWithCovarianceStamped(const rclcpp::PublisherBase::SharedPtr& pub,
  const Transformation& pose, const Eigen::Matrix<double, 6, 6>& covariance,
  const rclcpp::Time time, std::string frame_id);

// Publish pose with transform
void publishPoseWithTransform(const rclcpp::PublisherBase::SharedPtr& pub,
  tf2_ros::TransformBroadcaster& broadcaster,
  const Transformation& pose, const rclcpp::Time time,
  std::string frame_id, std::string child_frame_id);

// Publish pose with covariance and transform
void publishPoseWithCovarianceAndTransform(const rclcpp::PublisherBase::SharedPtr& pub,
  tf2_ros::TransformBroadcaster& broadcaster,
  const Transformation& pose, const Eigen::Matrix<double, 6, 6>& covariance,
  const rclcpp::Time time, std::string frame_id, std::string child_frame_id);

// Publish odometry
void publishOdometry(const rclcpp::PublisherBase::SharedPtr& pub,
  tf2_ros::TransformBroadcaster& broadcaster,
  const Transformation& pose, const Eigen::Vector3d& velocity,
  const Eigen::Matrix<double, 9, 9>& covariance, const rclcpp::Time time,
  std::string frame_id, std::string child_frame_id);

void publishNavSatFix(const rclcpp::PublisherBase::SharedPtr& pub, const Eigen::Vector3d& lla,
  Eigen::Matrix3d& covariance, const rclcpp::Time time, GnssSolutionStatus status);

// Path publisher
class PathPublisher {
public:
  // Add a pose and publish all previous poses
  void addPoseAndPublish(const rclcpp::PublisherBase::SharedPtr& pub,
    const Transformation& pose, const rclcpp::Time time,
    std::string frame_id);

  // Clear previous poses
  inline void clear() {
    path_.poses.clear();
    is_initialized_ = false;
  }

protected:
  bool is_initialized_ = false;
  nav_msgs::msg::Path path_;
};

// Publish 3D error
void publishError3d(const rclcpp::PublisherBase::SharedPtr& pub,
  const Eigen::Vector3d& error, const rclcpp::Time time,
  std::string frame_id);

// Publish IMU message
void publishImu(const rclcpp::PublisherBase::SharedPtr& pub, const DataCluster::IMU& imu);
void publishImu(const rclcpp::PublisherBase::SharedPtr& pub, const ImuMeasurement& imu);

// Publish GNSS message
void publishGnssObservations(
  const rclcpp::PublisherBase::SharedPtr& pub, const DataCluster::GNSS& gnss);
void publishGnssEphemerides(
  const rclcpp::PublisherBase::SharedPtr& pub, const DataCluster::GNSS& gnss);
void publishGnssAntennaPosition(
  const rclcpp::PublisherBase::SharedPtr& pub, const DataCluster::GNSS& gnss);
void publishGnssIonosphereParameter(
  const rclcpp::PublisherBase::SharedPtr& pub, const DataCluster::GNSS& gnss);
void publishGnssSsrCodeBiases(
  const rclcpp::PublisherBase::SharedPtr& pub, const DataCluster::GNSS& gnss);
void publishGnssSsrPhaseBiases(
  const rclcpp::PublisherBase::SharedPtr& pub, const DataCluster::GNSS& gnss);
void publishGnssSsrEphemerides(
  const rclcpp::PublisherBase::SharedPtr& pub, const DataCluster::GNSS& gnss);

}
