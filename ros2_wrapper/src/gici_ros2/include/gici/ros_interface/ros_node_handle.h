/**
* @Function: Handle ROS 2 streams
*
* Original author: Cheng Chi <chichengcn@sjtu.edu.cn>
* ROS 2 (rclcpp/ament) port for the gici_research_standard UrbanNav setup.
**/
#pragma once

#include <iostream>
#include <vector>
#include <unordered_map>
#include <functional>
#include <glog/logging.h>
#include <rclcpp/rclcpp.hpp>

#include "gici/stream/node_handle.h"
#include "gici/ros_interface/ros_stream.h"

namespace gici {

class RosNodeHandle : public NodeHandle {
public:
  RosNodeHandle(rclcpp::Node::SharedPtr node, const NodeOptionHandlePtr& nodes);
  ~RosNodeHandle();

protected:
  // Bind streamer->formator->ROS-streamer pipelines
  void bindStreamerToFormatorToRosStreamer(const NodeOptionHandlePtr& nodes);

  // Bind ROS-streamer->formator->streamer pipelines
  void bindRosStreamerToFormatorToStreamer(const NodeOptionHandlePtr& nodes);

  // Bind estimator->ROS-streamer pipelines
  void bindEstimatorToRosStreamer(const NodeOptionHandlePtr& nodes);

  // Clear streamer->formator->estimator pipeline and rebind them together with
  // ROS-streamer->estimator pipelines
  void rebindAllStreamerToEstimator(const NodeOptionHandlePtr& nodes);

  // Get ROS stream from tag
  inline std::shared_ptr<RosStream> getRosStreamFromTag(std::string tag) {
    for (auto ros_stream : ros_streams_) {
      if (ros_stream->getTag() == tag) return ros_stream;
    }
    return nullptr;
  }

protected:
  std::vector<std::shared_ptr<RosStream>> ros_streams_;
};

}
