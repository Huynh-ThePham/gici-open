cmake_minimum_required(VERSION 3.8)
project(gici_ros2_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GnssObservation.msg"
  "msg/GnssObservations.msg"
  "msg/GnssEphemeris.msg"
  "msg/GlonassEphemeris.msg"
  "msg/GnssEphemerides.msg"
  "msg/GnssAntennaPosition.msg"
  "msg/GnssIonosphereParameter.msg"
  "msg/GnssSsrCodeBias.msg"
  "msg/GnssSsrCodeBiases.msg"
  "msg/GnssSsrPhaseBias.msg"
  "msg/GnssSsrPhaseBiases.msg"
  "msg/GnssSsrEphemeris.msg"
  "msg/GnssSsrEphemerides.msg"
  DEPENDENCIES std_msgs
)

ament_package()
