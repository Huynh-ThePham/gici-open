/**
* @Function: main function of the ROS 2 wrapper of gici library
*
* Original author: Cheng Chi <chichengcn@sjtu.edu.cn>
* ROS 2 (rclcpp/ament) port for the gici_research_standard UrbanNav setup.
*
* Usage: ros2 run gici_ros2 gici_ros2_main <path-to-config>
**/
#include <csignal>
#include <rclcpp/rclcpp.hpp>

#include "gici/ros_interface/ros_node_handle.h"
#include "gici/utility/signal_handle.h"
#include "gici/utility/node_option_handle.h"
#include "gici/utility/spin_control.h"

using namespace gici;

namespace {
std::shared_ptr<rclcpp::Node> g_node;

void handleShutdownSignal(int)
{
  if (g_node) {
    rclcpp::shutdown();
  }
}
}  // namespace

int main(int argc, char** argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Get config file. ROS 2 injects its own args; keep the first non-ROS argument.
  std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);

  // Create the ROS 2 node (OpenVINS-style: config via CLI arg or ROS param)
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("gici", node_options);
  g_node = node;
  if (!node->has_parameter("config_file")) {
    node->declare_parameter<std::string>("config_file", "");
  }
  if (!node->has_parameter("use_sim_time")) {
    node->declare_parameter<bool>("use_sim_time", false);
  }

  std::string config_file_path;
  if (args.size() == 2) {
    config_file_path = args[1];
  } else {
    config_file_path = node->get_parameter("config_file").as_string();
  }
  if (config_file_path.empty()) {
    // node already exists at this point (unlike the pre-merge ros2-realtime-fix
    // version, where this check ran before Node creation and had to use a named
    // logger), so this can log through the node's own logger.
    RCLCPP_ERROR(node->get_logger(),
      "Invalid input! Provide config as: gici_ros2_main <path-to-config>, "
      "or ROS param config_file via launch / --params-file");
    rclcpp::shutdown();
    return -1;
  }

  std::signal(SIGINT, handleShutdownSignal);
  std::signal(SIGTERM, handleShutdownSignal);
  YAML::Node yaml_node;
  try {
     yaml_node = YAML::LoadFile(config_file_path);
  } catch (YAML::BadFile &e) {
    RCLCPP_ERROR(node->get_logger(), "Unable to load config file!");
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
    RCLCPP_ERROR(node->get_logger(), "Invalid configurations!");
    rclcpp::shutdown();
    return -1;
  }

  // Initialize nodes
  std::unique_ptr<RosNodeHandle> node_handle =
    std::make_unique<RosNodeHandle>(node, node_option_handle);

  // Show information. Logged via RCLCPP_INFO (not std::cout) so it appears on
  // /rosout and respects ros2 launch log-level filtering; the substring
  // "Initialized N streamers...Running" is still what
  // scripts/ros2/ros2_bag_replay_common.sh greps for readiness, and grep -E
  // is not anchored so the RCLCPP_INFO log prefix does not break that check.
  const std::vector<size_t> sizes = {
    node_option_handle->streamers.size(),
    node_option_handle->formators.size(),
    node_option_handle->estimators.size()};
  RCLCPP_INFO(node->get_logger(),
    "Initialized %zu streamer%s, %zu formater%s, and %zu estimator%s. Running...",
    sizes[0], sizes[0] > 1 ? "s" : "",
    sizes[1], sizes[1] > 1 ? "s" : "",
    sizes[2], sizes[2] > 1 ? "s" : "");

  // Start running all threads
  SpinControl::run();

  // Loop
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
