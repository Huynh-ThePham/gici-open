# GICI ROS 2 standard topic layout (OpenVINS-style interface)

All topics under `/gici/` namespace. Custom GNSS types remain in `gici_ros2_msgs`.

**Live mode:** configs with `replay.enable: false` and ROS input streamers — see
`ros2_wrapper/docs/ROS2_LIVE_NODE.md`.

## Inputs (bag replay / live)

| Topic | Type | Role |
|-------|------|------|
| `/gici/gnss_rover/observations` | `gici_ros2_msgs/GnssObservations` | Rover GNSS |
| `/gici/gnss_rover/ephemerides` | `gici_ros2_msgs/GnssEphemerides` | (if enabled) |
| `/gici/gnss_reference/observations` | `gici_ros2_msgs/GnssObservations` | Reference GNSS |
| `/gici/gnss_reference/antenna_position` | `gici_ros2_msgs/GnssAntennaPosition` | Ref position |
| `/gici/imu_raw` | `sensor_msgs/Imu` | IMU |
| `/gici/image_raw` | `sensor_msgs/Image` | Camera |

Ephemeris + DCB for GICI board hybrid bag replay load from file streamers (not ROS topics).

## Outputs (standard — enabled in postfile/bag yaml on this branch)

| Topic | Type | TF |
|-------|------|-----|
| `/gici/odom` | `nav_msgs/Odometry` | `World` → `base_link` |
| `/gici/path` | `nav_msgs/Path` | frame `World` |
| `/gici/pose` | `geometry_msgs/PoseStamped` | frame `World` |
| `OUTPUT_DIR/solution.txt` | NMEA file | author eval / baseline |

## Launch

```bash
# LIVE node (topic-driven fusion — GICI sống trong ROS2)
./scripts/ros2/launch_gici_live.sh board 1.1
./scripts/ros2/launch_gici_live.sh board 1.1 bag 1.0

# Postfile batch (NOT live — paper baseline)
./scripts/ros2/launch_gici_board_std.sh postfile 1.1

# Bag replay via std wrapper
./scripts/ros2/launch_gici_board_std.sh bag 1.1 1.0

# Low-level
ros2 launch gici_ros2 gici_live.launch.py config:=/tmp/live.yaml
```

## rviz

- Fixed frame: `World`
- Odometry: `/gici/odom`
- Path: `/gici/path`
