/**
* @Function: Handle ROS 2 stream publishing and subscribing
*
* Original author: Cheng Chi <chichengcn@sjtu.edu.cn>
* ROS 2 (rclcpp/ament) port for the gici_research_standard UrbanNav setup.
**/
#include "gici/ros_interface/ros_stream.h"

#include <functional>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

#include "gici/utility/spin_control.h"
#include "gici/gnss/gnss_common.h"

namespace gici {

using std::placeholders::_1;

// Static variables for stream binding
std::vector<RosStream *> RosStream::static_this_;

RosStream::RosStream(
  rclcpp::Node::SharedPtr node, const NodeOptionHandlePtr& nodes, int istreamer) :
  Streaming(), node_(node), frame_id_("World"), valid_(false)
{
  // Get streamer option
  const auto& streamer_node = nodes->streamers[istreamer];
  tag_ = streamer_node->tag;
  input_ros_stream_tag_.clear();
  for (const auto& input_tag : streamer_node->input_tags) {
    if (input_tag.substr(0, 4) != "str_") continue;
    if (nodes->tag_to_node.at(input_tag)->type != "ros") {
      LOG(ERROR) << "Only the ROS streamer can directly connect to a ROS streamer!"
        << " Invalid tag is " << input_tag << " in " << tag_;
      return;
    }
    if (input_ros_stream_tag_.empty()) input_ros_stream_tag_ = input_tag;
    else {
      LOG(ERROR) << "Only one input stream is supported for ROS stream!";
      return;
    }
  }
  if (!option_tools::safeGet(streamer_node->this_node, "topic_name", &topic_name_)) {
    LOG(ERROR) << "Unable to load ROS topic name!";
    return;
  }
  if (!option_tools::safeGet(streamer_node->this_node, "queue_size", &queue_size_)) {
    LOG(INFO) << "Unable to load ROS topic queue size. Using default instead";
    queue_size_ = 10;
  }
  std::string type_str;
  if (!option_tools::safeGet(streamer_node->this_node, "io", &type_str)) {
    LOG(ERROR) << "Unable to load ROS streamer I/O type!";
    return;
  }
  option_tools::convert(type_str, io_type_);
  if (io_type_ == StreamIOType::Log) io_type_ = StreamIOType::Output;
  if (io_type_ != StreamIOType::Input && io_type_ != StreamIOType::Output) {
    LOG(ERROR) << "Invalid IO type for ROS streamer!";
  }
  // Replaying a dataset must not drop GNSS/sensor messages regardless of playback
  // rate (GICI keys off the message week/tow, not the wall clock), so subscribe with
  // reliable + keep-all history. A small KeepLast queue would silently drop bursts of
  // ephemeris/observations at high rate and stall the estimator ("waiting for
  // ephemeris"). Outputs keep a bounded reliable queue.
  const rclcpp::QoS qos = (io_type_ == StreamIOType::Input)
    ? rclcpp::QoS(rclcpp::KeepAll()).reliable()
    : rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(queue_size_))).reliable();
  // initialize ros topic
  std::string data_format;
  if (!option_tools::safeGet(streamer_node->this_node, "format", &data_format)) {
    LOG(ERROR) << "Unable to load ROS topic format!";
    return;
  }
  if (data_format == "image") {
    data_format_ = RosDataFormat::Image;
    if (io_type_ == StreamIOType::Input) {
      subscribers_.push_back(node_->create_subscription<sensor_msgs::msg::Image>(
        topic_name_, qos, std::bind(&RosStream::imageCallback, this, _1)));
    }
    else {
      publishers_.push_back(node_->create_publisher<sensor_msgs::msg::Image>(topic_name_, qos));
    }
  }
  else if (data_format == "imu") {
    data_format_ = RosDataFormat::Imu;
    if (io_type_ == StreamIOType::Input) {
      subscribers_.push_back(node_->create_subscription<sensor_msgs::msg::Imu>(
        topic_name_, qos, std::bind(&RosStream::imuCallback, this, _1)));
    }
    else {
      publishers_.push_back(node_->create_publisher<sensor_msgs::msg::Imu>(topic_name_, qos));
    }
  }
  else if (data_format == "gnss_raw") {
    data_format_ = RosDataFormat::GnssRaw;
    if (io_type_ == StreamIOType::Input) {
      bool enable = false;
      if (option_tools::safeGet(streamer_node->this_node,
          "enable_observation", &enable)) {
        subscribers_.push_back(node_->create_subscription<gici_ros2_msgs::msg::GnssObservations>(
          topic_name_ + "/observations", qos,
          std::bind(&RosStream::gnssObservationsCallback, this, _1)));
        gnss_formats_.push_back(RosGnssDataFormat::Observations);
      }
      if (option_tools::safeGet(streamer_node->this_node,
          "enable_ephemeris", &enable)) {
        subscribers_.push_back(node_->create_subscription<gici_ros2_msgs::msg::GnssEphemerides>(
          topic_name_ + "/ephemerides", qos,
          std::bind(&RosStream::gnssEphemeridesCallback, this, _1)));
        gnss_formats_.push_back(RosGnssDataFormat::Ephemerides);
      }
      if (option_tools::safeGet(streamer_node->this_node,
          "enable_antenna_position", &enable)) {
        subscribers_.push_back(node_->create_subscription<gici_ros2_msgs::msg::GnssAntennaPosition>(
          topic_name_ + "/antenna_position", qos,
          std::bind(&RosStream::gnssAntennaPositionCallback, this, _1)));
        gnss_formats_.push_back(RosGnssDataFormat::AntennaPosition);
      }
      if (option_tools::safeGet(streamer_node->this_node,
          "enable_ionosphere_parameter", &enable)) {
        subscribers_.push_back(node_->create_subscription<gici_ros2_msgs::msg::GnssIonosphereParameter>(
          topic_name_ + "/ionosphere_parameter", qos,
          std::bind(&RosStream::gnssIonosphereParameterCallback, this, _1)));
        gnss_formats_.push_back(RosGnssDataFormat::IonosphereParameter);
      }
      if (option_tools::safeGet(streamer_node->this_node,
          "enable_ssr_code_bias", &enable)) {
        subscribers_.push_back(node_->create_subscription<gici_ros2_msgs::msg::GnssSsrCodeBiases>(
          topic_name_ + "/code_bias", qos,
          std::bind(&RosStream::gnssSsrCodeBiasesCallback, this, _1)));
        gnss_formats_.push_back(RosGnssDataFormat::CodeBias);
      }
      if (option_tools::safeGet(streamer_node->this_node,
          "enable_ssr_phase_bias", &enable)) {
        subscribers_.push_back(node_->create_subscription<gici_ros2_msgs::msg::GnssSsrPhaseBiases>(
          topic_name_ + "/phase_bias", qos,
          std::bind(&RosStream::gnssSsrPhaseBiasesCallback, this, _1)));
        gnss_formats_.push_back(RosGnssDataFormat::PhaseBias);
      }
      if (option_tools::safeGet(streamer_node->this_node,
          "enable_ssr_ephemeris", &enable)) {
        subscribers_.push_back(node_->create_subscription<gici_ros2_msgs::msg::GnssSsrEphemerides>(
          topic_name_ + "/ephemerides_correction", qos,
          std::bind(&RosStream::gnssSsrEphemeridesCallback, this, _1)));
        gnss_formats_.push_back(RosGnssDataFormat::EphemeridesCorrection);
      }
    }
    else {
      bool enable = false;
      if (option_tools::safeGet(streamer_node->this_node,
          "enable_observation", &enable)) {
        publishers_.push_back(node_->create_publisher<gici_ros2_msgs::msg::GnssObservations>(
          topic_name_ + "/observations", qos));
        gnss_formats_.push_back(RosGnssDataFormat::Observations);
      }
      if (option_tools::safeGet(streamer_node->this_node,
          "enable_ephemeris", &enable)) {
        publishers_.push_back(node_->create_publisher<gici_ros2_msgs::msg::GnssEphemerides>(
          topic_name_ + "/ephemerides", qos));
        gnss_formats_.push_back(RosGnssDataFormat::Ephemerides);
      }
      if (option_tools::safeGet(streamer_node->this_node,
          "enable_antenna_position", &enable)) {
        publishers_.push_back(node_->create_publisher<gici_ros2_msgs::msg::GnssAntennaPosition>(
          topic_name_ + "/antenna_position", qos));
        gnss_formats_.push_back(RosGnssDataFormat::AntennaPosition);
      }
      if (option_tools::safeGet(streamer_node->this_node,
          "enable_ionosphere_parameter", &enable)) {
        publishers_.push_back(node_->create_publisher<gici_ros2_msgs::msg::GnssIonosphereParameter>(
          topic_name_ + "/ionosphere_parameter", qos));
        gnss_formats_.push_back(RosGnssDataFormat::IonosphereParameter);
      }
      if (option_tools::safeGet(streamer_node->this_node,
          "enable_ssr_code_bias", &enable)) {
        publishers_.push_back(node_->create_publisher<gici_ros2_msgs::msg::GnssSsrCodeBiases>(
          topic_name_ + "/code_bias", qos));
        gnss_formats_.push_back(RosGnssDataFormat::CodeBias);
      }
      if (option_tools::safeGet(streamer_node->this_node,
          "enable_ssr_phase_bias", &enable)) {
        publishers_.push_back(node_->create_publisher<gici_ros2_msgs::msg::GnssSsrPhaseBiases>(
          topic_name_ + "/phase_bias", qos));
        gnss_formats_.push_back(RosGnssDataFormat::PhaseBias);
      }
      if (option_tools::safeGet(streamer_node->this_node,
          "enable_ssr_ephemeris", &enable)) {
        publishers_.push_back(node_->create_publisher<gici_ros2_msgs::msg::GnssSsrEphemerides>(
          topic_name_ + "/ephemerides_correction", qos));
        gnss_formats_.push_back(RosGnssDataFormat::EphemeridesCorrection);
      }
    }
  }
  else if (data_format == "pose_stamped") {
    data_format_ = RosDataFormat::PoseStamped;
    if (io_type_ == StreamIOType::Input) {
      LOG(ERROR) << "Setting pose stamped topic as input is disabled! If you want to input pose"
                 << ", use \"pose_with_covariance_stamped\" instead.";
      return;
    }
    else {
      publishers_.push_back(node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name_, qos));
      // publish transform if sub-frame is specified
      if (option_tools::safeGet(streamer_node->this_node, "subframe_id", &subframe_id_)) {
        tranform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
      }
    }
  }
  else if (data_format == "pose_with_covariance_stamped") {
    data_format_ = RosDataFormat::PoseWithCovarianceStamped;
    if (io_type_ == StreamIOType::Input) {
      subscribers_.push_back(node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        topic_name_, qos, std::bind(&RosStream::poseCallback, this, _1)));
      // we need a coordinate zero for local frame to global frame convertion
      std::vector<double> initial_global_position_vec;
      if (option_tools::safeGet(streamer_node->this_node,
        "initial_global_position", &initial_global_position_vec)) {
        CHECK(initial_global_position_vec.size() == 3);
        Eigen::Vector3d initial_global_position;
        for (size_t i = 0; i < 3; i++) {
          initial_global_position(i) = initial_global_position_vec[i];
        }
        initial_global_position.head<2>() *= D2R;
        input_coordinate_.reset(new GeoCoordinate(initial_global_position, GeoType::LLA));
      }
      else {
        LOG(ERROR) << "A initial_global_position is required for the "
                   << "pose_with_covariance_stamped topic in input mode!";
        return;
      }
    }
    else {
      publishers_.push_back(node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        topic_name_, qos));
      // publish transform if sub-frame is specified
      if (option_tools::safeGet(streamer_node->this_node, "subframe_id", &subframe_id_)) {
        tranform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
      }
    }
  }
  else if (data_format == "odometry") {
    data_format_ = RosDataFormat::Odometry;
    if (io_type_ == StreamIOType::Input) {
      LOG(ERROR) << "Setting odometry topic as input is disabled!";
      return;
    }
    else {
      publishers_.push_back(node_->create_publisher<nav_msgs::msg::Odometry>(
        topic_name_, qos));
      // publish transform if sub-frame is specified
      if (option_tools::safeGet(streamer_node->this_node, "subframe_id", &subframe_id_)) {
        tranform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
      }
      else {
        LOG(ERROR) << "A subframe_id is required for the odometry topic!";
        return;
      }
    }
  }
  else if (data_format == "navsatfix") {
    data_format_ = RosDataFormat::NavSatFix;
    if (io_type_ == StreamIOType::Input) {
      subscribers_.push_back(node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        topic_name_, qos, std::bind(&RosStream::navSatFixCallback, this, _1)));
    }
    else {
      publishers_.push_back(node_->create_publisher<sensor_msgs::msg::NavSatFix>(
        topic_name_, qos));
    }
  }
  else if (data_format == "marker") {
    data_format_ = RosDataFormat::Marker;
    if (io_type_ == StreamIOType::Input) {
      LOG(ERROR) << "Setting marker topic as input is disabled!";
      return;
    }
    else {
      option_tools::safeGet(streamer_node->this_node, "marker_scale", &marker_scale_);
      publishers_.push_back(node_->create_publisher<visualization_msgs::msg::Marker>(topic_name_, qos));
    }
  }
  else if (data_format == "path") {
    data_format_ = RosDataFormat::Path;
    if (io_type_ == StreamIOType::Input) {
      LOG(ERROR) << "Setting path topic as input is disabled!";
      return;
    }
    else {
      publishers_.push_back(node_->create_publisher<nav_msgs::msg::Path>(topic_name_, qos));
    }
    path_publisher_ = std::make_unique<PathPublisher>();
  }

  // Set valid
  valid_ = true;

  // Save to global for binding
  static_this_.push_back(this);
}

RosStream::~RosStream()
{}

// Output data callback
void RosStream::outputDataCallback(
    const std::string tag, const std::shared_ptr<DataCluster>& data)
{
  if (data->solution) {
    solutionOutputCallback(tag, *data->solution);
  }
  else if (data->frame) {
    featuredImageOutputCallback(data->frame);
  }
  else if (data->map) {
    mapPointOutputCallback(data->map);
  }
  else if (data->gnss) {
    gnssRawDataOutputCallback(*data->gnss);
  }
  else if (data->imu) {
    imuDataOutputCallback(*data->imu);
  }
  else if (data->image) {
    imageDataOutputCallback(*data->image);
  }
}

// Pipeline sends input data to logging stream
void RosStream::pipelineCallback(
  const std::string& tag, const std::shared_ptr<DataCluster>& data)
{
  outputDataCallback(tag, data);
}

// Bind input and logging streams (ROS to ROS)
void RosStream::bindLogWithInput()
{
  for (size_t i = 0; i < static_this_.size(); i++) {
    if (static_this_[i]->input_ros_stream_tag_.empty()) continue;
    auto& streaming_log = static_this_[i];
    std::string& input_tag = streaming_log->input_ros_stream_tag_;
    for (size_t j = 0; j < static_this_.size(); j++) {
      if (static_this_[j]->tag_ != input_tag) continue;
      auto& streaming_in = static_this_[j];
      PipelineCallback pipeline = std::bind(&RosStream::pipelineCallback,
        streaming_log, std::placeholders::_1, std::placeholders::_2);
      streaming_in->pipeline_ros_to_ros_.push_back(pipeline);
    }
  }
}

// Send solution data to ROS topic
void RosStream::solutionOutputCallback(std::string tag, Solution& solution)
{
  if (data_format_ == RosDataFormat::PoseStamped) {
    if (tranform_broadcaster_) {
      publishPoseWithTransform(publishers_[0], *tranform_broadcaster_, solution.pose,
        toRosTime(solution.timestamp), frame_id_, subframe_id_);
    }
    else {
      publishPoseStamped(publishers_[0], solution.pose,
        toRosTime(solution.timestamp), frame_id_);
    }
  }
  else if (data_format_ == RosDataFormat::PoseWithCovarianceStamped) {
    if (tranform_broadcaster_) {
      publishPoseWithCovarianceAndTransform(publishers_[0], *tranform_broadcaster_,
        solution.pose, solution.covariance.block<6, 6>(0, 0),
        toRosTime(solution.timestamp), frame_id_, subframe_id_);
    }
    else {
      publishPoseWithCovarianceStamped(publishers_[0],
        solution.pose, solution.covariance.block<6, 6>(0, 0),
        toRosTime(solution.timestamp), frame_id_);
    }
  }
  else if (data_format_ == RosDataFormat::Odometry) {
    CHECK_NOTNULL(tranform_broadcaster_);
    publishOdometry(publishers_[0], *tranform_broadcaster_, solution.pose,
      solution.speed_and_bias.head<3>(), solution.covariance.block<9, 9>(0, 0),
      toRosTime(solution.timestamp), frame_id_, subframe_id_);
  }
  else if (data_format_ == RosDataFormat::NavSatFix) {
    Eigen::Vector3d lla =
      solution.coordinate->convert(
        solution.pose.getPosition(), GeoType::ENU, GeoType::LLA);
    Eigen::Matrix3d lla_covariance =
      solution.coordinate->convertCovariance(
        solution.covariance.topLeftCorner(3, 3), GeoType::ENU, GeoType::NED);
    publishNavSatFix(publishers_[0], lla,
      lla_covariance, toRosTime(solution.timestamp), solution.status);
  }
  else if (data_format_ == RosDataFormat::Path) {
    CHECK_NOTNULL(path_publisher_);
    path_publisher_->addPoseAndPublish(publishers_[0], solution.pose,
      toRosTime(solution.timestamp), frame_id_);
  }
}

// Send featured image to ROS topic
void RosStream::featuredImageOutputCallback(FramePtr& frame)
{
  if (data_format_ == RosDataFormat::Image) {
    publishFeaturedImage(publishers_[0], frame, toRosTime(frame->getTimestampSec()));
  }
}

// Send features as marker to ROS topic
void RosStream::mapPointOutputCallback(MapPtr& map)
{
  if (data_format_ == RosDataFormat::Marker) {
    if (map->numKeyframes() == 0) return;
    publishLandmarks(publishers_[0], map,
      toRosTime(map->getLastKeyframe()->getTimestampSec()), frame_id_, marker_scale_);
  }
}

// Send GNSS raw data to ROS topics
void RosStream::gnssRawDataOutputCallback(DataCluster::GNSS& gnss)
{
  if (data_format_ == RosDataFormat::GnssRaw) {
    std::vector<RosGnssDataFormat>::iterator format = gnss_formats_.end();
    for (auto type : gnss.types) {
      if (type == GnssDataType::Observation &&
          ((format = std::find(gnss_formats_.begin(), gnss_formats_.end(),
          RosGnssDataFormat::Observations)) != gnss_formats_.end())) {
        size_t i = format - gnss_formats_.begin();
        publishGnssObservations(publishers_[i], gnss);
      }
      else if (type == GnssDataType::Ephemeris &&
          ((format = std::find(gnss_formats_.begin(), gnss_formats_.end(),
          RosGnssDataFormat::Ephemerides)) != gnss_formats_.end())) {
        size_t i = format - gnss_formats_.begin();
        publishGnssEphemerides(publishers_[i], gnss);
      }
      else if (type == GnssDataType::AntePos &&
          ((format = std::find(gnss_formats_.begin(), gnss_formats_.end(),
          RosGnssDataFormat::AntennaPosition)) != gnss_formats_.end())) {
        size_t i = format - gnss_formats_.begin();
        publishGnssAntennaPosition(publishers_[i], gnss);
      }
      else if (type == GnssDataType::IonAndUtcPara &&
          ((format = std::find(gnss_formats_.begin(), gnss_formats_.end(),
          RosGnssDataFormat::IonosphereParameter)) != gnss_formats_.end())) {
        size_t i = format - gnss_formats_.begin();
        publishGnssIonosphereParameter(publishers_[i], gnss);
      }
      else if (type == GnssDataType::SSR) {
        if ((format = std::find(gnss_formats_.begin(), gnss_formats_.end(),
            RosGnssDataFormat::CodeBias)) != gnss_formats_.end()) {
          size_t i = format - gnss_formats_.begin();
          publishGnssSsrCodeBiases(publishers_[i], gnss);
        }
        if ((format = std::find(gnss_formats_.begin(), gnss_formats_.end(),
            RosGnssDataFormat::PhaseBias)) != gnss_formats_.end()) {
          size_t i = format - gnss_formats_.begin();
          publishGnssSsrPhaseBiases(publishers_[i], gnss);
        }
        if ((format = std::find(gnss_formats_.begin(), gnss_formats_.end(),
            RosGnssDataFormat::EphemeridesCorrection)) != gnss_formats_.end()) {
          size_t i = format - gnss_formats_.begin();
          publishGnssSsrEphemerides(publishers_[i], gnss);
        }
      }
    }
  }
}

// Send IMU data to ROS topic
void RosStream::imuDataOutputCallback(DataCluster::IMU& imu)
{
  if (data_format_ == RosDataFormat::Imu) {
    publishImu(publishers_[0], imu);
  }
}

// Send image data to ROS topic
void RosStream::imageDataOutputCallback(DataCluster::Image& image)
{
  if (data_format_ == RosDataFormat::Image) {
    cv::Mat cv_iamge(image.height, image.width, CV_8UC1, image.image);
    publishImage(publishers_[0], cv_iamge, toRosTime(image.time));
  }
}

// ROS callbacks
void RosStream::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  // Note that we set MONO8 as default, if the input image is colored image,
  // you should add a convertion here
  cv_bridge::CvImageConstPtr ptr =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  cv::Mat image = ptr->image;
  double timestamp = rclcpp::Time(msg->header.stamp).seconds();
  std::shared_ptr<DataCluster> data_cluster =
    std::make_shared<DataCluster>(FormatorType::ImagePack, image.cols, image.rows, 1);
  data_cluster->image->time = timestamp;
  memcpy(data_cluster->image->image, image.data, image.cols * image.rows);

  // Call Image processor
  for (auto it_image_callback : data_callbacks_) {
    it_image_callback(tag_, data_cluster);
  }

  // Call logger pipeline
  for (auto pipeline : pipeline_ros_to_ros_) {
    pipeline("", data_cluster);
  }
}

void RosStream::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  // Convert IMU data
  std::shared_ptr<DataCluster> data_cluster =
    std::make_shared<DataCluster>(FormatorType::IMUPack);
  data_cluster->imu->time = rclcpp::Time(msg->header.stamp).seconds();
  data_cluster->imu->acceleration[0] = msg->linear_acceleration.x;
  data_cluster->imu->acceleration[1] = msg->linear_acceleration.y;
  data_cluster->imu->acceleration[2] = msg->linear_acceleration.z;
  data_cluster->imu->angular_velocity[0] = msg->angular_velocity.x;
  data_cluster->imu->angular_velocity[1] = msg->angular_velocity.y;
  data_cluster->imu->angular_velocity[2] = msg->angular_velocity.z;

  // Call INS processor
  for (auto it_imu_callback : data_callbacks_) {
    it_imu_callback(tag_, data_cluster);
  }

  // Call logger pipeline
  for (auto pipeline : pipeline_ros_to_ros_) {
    pipeline("", data_cluster);
  }
}

void RosStream::gnssObservationsCallback(
  const gici_ros2_msgs::msg::GnssObservations::ConstSharedPtr msg)
{
  // Convert observation data
  std::shared_ptr<DataCluster> data_cluster =
    std::make_shared<DataCluster>(FormatorType::GnssRaw);
  data_cluster->gnss->observation->n = 0;
  // obsd_t only has NFREQ+NEXOBS frequency slots. Some datasets (e.g. UrbanNav)
  // publish more frequencies per satellite than the core is compiled for; writing
  // past the slot count corrupts the heap and later crashes in DataCluster::free().
  // Clamp to the number of slots the core actually supports.
  const size_t max_freq = NFREQ + NEXOBS;
  for (const auto& o : msg->observations) {
    int n = data_cluster->gnss->observation->n;
    if (n >= MAXOBS) break;
    obsd_t *obs = data_cluster->gnss->observation->data + n;
    memset(obs, 0, sizeof(obsd_t));
    obs->time = gpst2time(o.week, o.tow);
    obs->sat = satid2no(o.prn.data());
    obs->rcv = 0;
    for (size_t i = 0; i < o.code.size() && i < max_freq; i++) {
      obs->SNR[i] = o.snr[i];
      obs->LLI[i] = o.lli[i];
      obs->code[i] = gnss_common::rinexTypeToCodeType(o.prn[0], o.code[i]);
      obs->L[i] = o.l[i];
      obs->P[i] = o.p[i];
      obs->D[i] = o.d[i];
    }
    data_cluster->gnss->observation->n++;
  }
  sortobs(data_cluster->gnss->observation);
  data_cluster->gnss->types.push_back(GnssDataType::Observation);

  // Call GNSS processor
  for (auto it_gnss_callback : data_callbacks_) {
    it_gnss_callback(tag_, data_cluster);
  }

  // Call logger pipeline
  for (auto pipeline : pipeline_ros_to_ros_) {
    pipeline("", data_cluster);
  }
}
void RosStream::gnssEphemeridesCallback(
  const gici_ros2_msgs::msg::GnssEphemerides::ConstSharedPtr msg)
{
  // Convert ephemerides data
  std::shared_ptr<DataCluster> data_cluster =
    std::make_shared<DataCluster>(FormatorType::GnssRaw);
  static std::unique_ptr<raw_t> raw_ = nullptr;
  if (raw_ == nullptr) {
    raw_ = std::make_unique<raw_t>();
    init_raw(raw_.get(), 0);
  }
  nav_t *nav = &raw_->nav;
  for (const auto& e : msg->ephemerides) {
    eph_t eph = {0};
    eph.sat = satid2no(e.prn.data());
    eph.week = e.week;
    if (e.prn[0] == 'C') {
      eph.toe = bdt2gpst(bdt2time(e.week, e.toes));
      eph.toc = bdt2gpst(bdt2time(e.week, e.toc));
    }
    else {
      eph.toe = gpst2time(e.week, e.toes);
      eph.toc = gpst2time(e.week, e.toc);
    }
    eph.A = e.a;
    eph.sva = e.sva;
    eph.code = e.code;
    eph.idot = e.idot;
    eph.iode = e.iode;
    eph.f2 = e.f2;
    eph.f1 = e.f1;
    eph.f0 = e.f0;
    eph.iodc = e.iodc;
    eph.crs = e.crs;
    eph.deln = e.deln;
    eph.M0 = e.m0;
    eph.cuc = e.cuc;
    eph.e = e.e;
    eph.cus = e.cus;
    eph.toes = e.toes;
    eph.cic = e.cic;
    eph.OMG0 = e.omg0;
    eph.cis = e.cis;
    eph.i0 = e.i0;
    eph.crc = e.crc;
    eph.omg = e.omg;
    eph.OMGd = e.omgd;
    const size_t max_tgd = sizeof(eph.tgd) / sizeof(eph.tgd[0]);
    for (size_t i = 0; i < e.tgd.size() && i < max_tgd; i++) eph.tgd[i] = e.tgd[i];
    eph.svh = e.svh;
    nav->eph[eph.sat - 1] = eph;
    gnss_common::updateEphemeris(nav, eph.sat, data_cluster->gnss);
  }
  for (const auto& e : msg->glonass_ephemerides) {
    geph_t geph= {0};
    geph.sat = satid2no(e.prn.data());
    geph.svh = e.svh;
    geph.iode = e.iode;
    geph.tof = gpst2time(e.week, e.tof);
    geph.toe = gpst2time(e.week, e.toe);
    geph.frq = e.frq;
    for (int i = 0; i < 3; i++) {
      geph.vel[i] = e.vel[i];
      geph.pos[i] = e.pos[i];
      geph.acc[i] = e.acc[i];
    }
    geph.gamn = e.gamn;
    geph.taun = e.taun;
    geph.dtaun = e.dtaun;
    geph.age = e.age;
    nav->geph[atoi(e.prn.substr(1, 2).data()) - 1] = geph;
    gnss_common::updateEphemeris(nav, geph.sat, data_cluster->gnss);
  }
  data_cluster->gnss->types.push_back(GnssDataType::Ephemeris);

  // Call GNSS processor
  for (auto it_gnss_callback : data_callbacks_) {
    it_gnss_callback(tag_, data_cluster);
  }

  // Call logger pipeline
  for (auto pipeline : pipeline_ros_to_ros_) {
    pipeline("", data_cluster);
  }
}
void RosStream::gnssAntennaPositionCallback(
  const gici_ros2_msgs::msg::GnssAntennaPosition::ConstSharedPtr msg)
{
  // Convert antenna data
  std::shared_ptr<DataCluster> data_cluster =
    std::make_shared<DataCluster>(FormatorType::GnssRaw);
  for (size_t i = 0; i < 3; i++) {
    data_cluster->gnss->antenna->pos[i] = msg->pos[i];
  }
  data_cluster->gnss->types.push_back(GnssDataType::AntePos);

  // Call GNSS processor
  for (auto it_gnss_callback : data_callbacks_) {
    it_gnss_callback(tag_, data_cluster);
  }

  // Call logger pipeline
  for (auto pipeline : pipeline_ros_to_ros_) {
    pipeline("", data_cluster);
  }
}
void RosStream::gnssIonosphereParameterCallback(
  const gici_ros2_msgs::msg::GnssIonosphereParameter::ConstSharedPtr msg)
{
  // Convert ionosphere data
  std::shared_ptr<DataCluster> data_cluster =
    std::make_shared<DataCluster>(FormatorType::GnssRaw);
  if (msg->type == 0) {
    for (int i = 0; i < 8; i++) {
      data_cluster->gnss->ephemeris->ion_gps[i] = msg->parameters[i];
    }
  }
  else if (msg->type == 1) {
    for (int i = 0; i < 8; i++) {
      data_cluster->gnss->ephemeris->ion_cmp[i] = msg->parameters[i];
    }
  }
  else if (msg->type == 2) {
    for (int i = 0; i < 4; i++) {
      data_cluster->gnss->ephemeris->ion_gal[i] = msg->parameters[i];
    }
  }
  data_cluster->gnss->types.push_back(GnssDataType::IonAndUtcPara);

  // Call GNSS processor
  for (auto it_gnss_callback : data_callbacks_) {
    it_gnss_callback(tag_, data_cluster);
  }

  // Call logger pipeline
  for (auto pipeline : pipeline_ros_to_ros_) {
    pipeline("", data_cluster);
  }
}
void RosStream::gnssSsrCodeBiasesCallback(
  const gici_ros2_msgs::msg::GnssSsrCodeBiases::ConstSharedPtr msg)
{
  // Convert code bias data
  std::shared_ptr<DataCluster> data_cluster =
    std::make_shared<DataCluster>(FormatorType::GnssRaw);
  for (const auto& b : msg->biases) {
    int sat = satid2no(b.prn.data());
    ssr_t *ssr = data_cluster->gnss->ephemeris->ssr + sat - 1;
    ssr->t0[4] = gpst2time(b.week, b.tow);
    ssr->udi[4] = b.udi;
    ssr->isdcb = b.isdcb;
    for (size_t i = 0; i < b.code.size(); i++) {
      int code = gnss_common::rinexTypeToCodeType(b.prn[0], b.code[i]);
      ssr->cbias[code - 1] = b.bias[i];
    }
    ssr->update = 1;
  }
  data_cluster->gnss->types.push_back(GnssDataType::SSR);

  // Call GNSS processor
  for (auto it_gnss_callback : data_callbacks_) {
    it_gnss_callback(tag_, data_cluster);
  }

  // Call logger pipeline
  for (auto pipeline : pipeline_ros_to_ros_) {
    pipeline("", data_cluster);
  }
}
void RosStream::gnssSsrPhaseBiasesCallback(
  const gici_ros2_msgs::msg::GnssSsrPhaseBiases::ConstSharedPtr msg)
{
  // Convert phase bias data
  std::shared_ptr<DataCluster> data_cluster =
    std::make_shared<DataCluster>(FormatorType::GnssRaw);
  for (const auto& b : msg->biases) {
    int sat = satid2no(b.prn.data());
    ssr_t *ssr = data_cluster->gnss->ephemeris->ssr + sat - 1;
    ssr->t0[4] = gpst2time(b.week, b.tow);
    ssr->udi[4] = b.udi;
    ssr->isdpb = b.isdpb;
    for (size_t i = 0; i < b.phase.size(); i++) {
      int phase = gnss_common::phaseStringToPhaseType(b.prn[0], b.phase[i]);
      int code = 0;
      for (; code < MAXCODE; code++) {
        if (gnss_common::getPhaseID(b.prn[0], code) == phase) break;
      }
      CHECK(code < MAXCODE);
      ssr->pbias[code - 1] = b.bias[i];
    }
    ssr->update = 1;
  }
  data_cluster->gnss->types.push_back(GnssDataType::SSR);

  // Call GNSS processor
  for (auto it_gnss_callback : data_callbacks_) {
    it_gnss_callback(tag_, data_cluster);
  }

  // Call logger pipeline
  for (auto pipeline : pipeline_ros_to_ros_) {
    pipeline("", data_cluster);
  }
}
void RosStream::gnssSsrEphemeridesCallback(
  const gici_ros2_msgs::msg::GnssSsrEphemerides::ConstSharedPtr msg)
{
  // Convert ephemeris correction data
  std::shared_ptr<DataCluster> data_cluster =
    std::make_shared<DataCluster>(FormatorType::GnssRaw);
  for (const auto& c : msg->corrections) {
    int sat = satid2no(c.prn.data());
    ssr_t *ssr = data_cluster->gnss->ephemeris->ssr + sat - 1;
    for (int i = 0; i < 2; i++) {
      ssr->t0[i] = gpst2time(c.week, c.tow);
      ssr->udi[i] = c.udi;
      ssr->iod[i] = c.iod;
    }
    ssr->iode = c.iode;
    ssr->iodcrc = c.iodcrc;
    ssr->refd = c.refd;
    for (int i = 0; i < 3; i++) {
      ssr->deph[i] = c.deph[i];
      ssr->ddeph[i] = c.ddeph[i];
      ssr->dclk[i] = c.dclk[i];
    }
    ssr->update = 1;
  }
  data_cluster->gnss->types.push_back(GnssDataType::SSR);

  // Call GNSS processor
  for (auto it_gnss_callback : data_callbacks_) {
    it_gnss_callback(tag_, data_cluster);
  }

  // Call logger pipeline
  for (auto pipeline : pipeline_ros_to_ros_) {
    pipeline("", data_cluster);
  }
}

void RosStream::poseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  // Convert data
  Solution solution;
  solution.timestamp = rclcpp::Time(msg->header.stamp).seconds();
  solution.coordinate = input_coordinate_;
  Eigen::Vector3d p(msg->pose.pose.position.x,
    msg->pose.pose.position.y, msg->pose.pose.position.z);
  Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  solution.pose = Transformation(p, q);
  solution.covariance.setZero();
  for (size_t i = 0; i < 6; i++) {
    for (size_t j = 0; j < 6; j++) {
      solution.covariance(i, j) = msg->pose.covariance[i * 6 + j];
    }
    CHECK(solution.covariance(i, i) > 0.0);
  }
  std::shared_ptr<DataCluster> data_cluster = std::make_shared<DataCluster>(solution);

  // Call other loosely couple estimators (internal pipeline)
  for (auto it_solution_callback : data_callbacks_) {
    it_solution_callback(tag_, data_cluster);
  }

  // Call logger pipeline
  for (auto pipeline : pipeline_ros_to_ros_) {
    pipeline("", data_cluster);
  }
}

void RosStream::navSatFixCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  // Convert data
  Solution solution;
  solution.timestamp = rclcpp::Time(msg->header.stamp).seconds();
  Eigen::Vector3d lla;
  lla(0) = msg->latitude * D2R;
  lla(1) = msg->longitude * D2R;
  lla(2) = msg->altitude;
  if (input_coordinate_ == nullptr) {
    input_coordinate_.reset(new GeoCoordinate(lla, GeoType::LLA));
  }
  solution.coordinate = input_coordinate_;
  solution.pose = Transformation(
    input_coordinate_->convert(lla, GeoType::LLA, GeoType::ENU),
    Eigen::Quaterniond::Identity());
  if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
    solution.status = GnssSolutionStatus::Fixed;
  }
  else {
    solution.status = GnssSolutionStatus::Float;
  }
  solution.covariance.setZero();
  if (msg->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
    for (size_t i = 0; i < 3; i++) {
      for (size_t j = 0; j < 3; j++) {
        solution.covariance(i, j) = msg->position_covariance[j + i * 3];
      }
    }
  }
  else {
    if (solution.status == GnssSolutionStatus::Fixed) {
      for (size_t i = 0; i < 3; i++) solution.covariance(i, i) = 1.0e-4;
    }
    else {
      for (size_t i = 0; i < 3; i++) solution.covariance(i, i) = 1.0e2;
    }
  }
  std::shared_ptr<DataCluster> data_cluster = std::make_shared<DataCluster>(solution);

  // Call other loosely couple estimators (internal pipeline)
  for (auto it_solution_callback : data_callbacks_) {
    it_solution_callback(tag_, data_cluster);
  }

  // Call logger pipeline
  for (auto pipeline : pipeline_ros_to_ros_) {
    pipeline("", data_cluster);
  }
}

}
