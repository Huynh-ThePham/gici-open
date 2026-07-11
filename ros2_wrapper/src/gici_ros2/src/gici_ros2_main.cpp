/**
* @Function: main function of the ROS 2 wrapper of gici library
*
* Original author: Cheng Chi <chichengcn@sjtu.edu.cn>
* ROS 2 (rclcpp/ament) port for the gici_research_standard UrbanNav setup.
*
* Usage: ros2 run gici_ros2 gici_ros2_main <path-to-config>
**/
#include <rclcpp/rclcpp.hpp>

#include "gici/ros_interface/ros_node_handle.h"
#include "gici/utility/signal_handle.h"
#include "gici/utility/node_option_handle.h"
#include "gici/utility/spin_control.h"

using namespace gici;

int main(int argc, char** argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Get config file. ROS 2 injects its own args; keep the first non-ROS argument.
  std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
  if (args.size() != 2) {
    std::cerr << "Invalid input variables! Supported variables are: "
              << "<path-to-executable> <path-to-config>" << std::endl;
    rclcpp::shutdown();
    return -1;
  }
  std::string config_file_path = args[1];
  YAML::Node yaml_node;
  try {
     yaml_node = YAML::LoadFile(config_file_path);
  } catch (YAML::BadFile &e) {
    std::cerr << "Unable to load config file!" << std::endl;
    rclcpp::shutdown();
    return -1;
  }

  // Initialize glog for logging
  bool enable_logging = false;
  if (yaml_node["logging"].IsDefined() &&
      option_tools::safeGet(yaml_node["logging"], "enable", &enable_logging) &&
      enable_logging == true) {
    YAML::Node logging_node = yaml_node["logging"];
    google::InitGoogleLogging("gici");
    int min_log_level = 0;
    if (option_tools::safeGet(
        logging_node, "min_log_level", &min_log_level)) {
      FLAGS_minloglevel = min_log_level;
    }
    option_tools::safeGet(logging_node, "log_to_stderr", &FLAGS_logtostderr);
    option_tools::safeGet(logging_node, "file_directory", &FLAGS_log_dir);
    if (FLAGS_logtostderr) FLAGS_stderrthreshold = min_log_level;
    else FLAGS_stderrthreshold = 5;
  }

  // Initialize signal handles to catch faults
  initializeSignalHandles();

  // Organize nodes
  NodeOptionHandlePtr node_option_handle =
    std::make_shared<NodeOptionHandle>(yaml_node);
  if (!node_option_handle->valid) {
    std::cerr << "Invalid configurations!" << std::endl;
    rclcpp::shutdown();
    return -1;
  }

  // Create the ROS 2 node
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("gici");

  // Initialize nodes
  std::unique_ptr<RosNodeHandle> node_handle =
    std::make_unique<RosNodeHandle>(node, node_option_handle);

  // Show information
  const std::vector<size_t> sizes = {
    node_option_handle->streamers.size(),
    node_option_handle->formators.size(),
    node_option_handle->estimators.size()};
  std::cout << "Initialized "
    << sizes[0] << " streamer" << (sizes[0] > 1 ? "s" : "") << ", "
    << sizes[1] << " formater" << (sizes[1] > 1 ? "s" : "") << ", and "
    << sizes[2] << " estimator" << (sizes[2] > 1 ? "s" : "") << ". "
    << "Running..." << std::endl;

  // Start running all threads
  SpinControl::run();

  // Loop
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
