# GICI sống trong ROS 2 — kiến trúc live node

GICI **đã có thể** chạy như một node ROS 2 thực sự: thuật toán fusion nhận dữ liệu
từ topic, xử lý đa luồng, và publish kết quả lên topic — không cần đọc file `.bin`
theo kiểu batch.

## Hai chế độ vận hành

| | **Post-file (batch)** | **Live (ROS2-native)** |
|---|---|---|
| Streamer | `type: post-file` | `type: ros`, `io: input` |
| `replay.enable` | `true` | `false` |
| Estimator | `MultiSensorSingleThreadEstimating` | `MultiSensorEstimating` |
| Dữ liệu vào | Đọc hết file, feed nhanh nhất có thể | Callback subscription qua `rclcpp::spin` |
| Mục đích | Baseline paper / tái lập số | Robot thật, bag replay realtime, RViz |

**Live = thuật toán sống trong ROS2.** Post-file chỉ dùng để so sánh baseline.

## Luồng dữ liệu (live)

```
Sensor publishers          GICI node (gici_ros2_main)
─────────────────         ─────────────────────────────
/gici/gnss_rover    ──►   RosStream (subscribe)
/gici/gnss_reference       │
/gici/imu_raw              ▼
/gici/image_raw         DataCluster → Estimator threads
                               │
                               ▼
                         /gici/odom  (nav_msgs/Odometry + TF)
                         /gici/path  (nav_msgs/Path)
                         /gici/pose  (geometry_msgs/PoseStamped)
```

Fusion chạy trên thread riêng; `rclcpp::spin` chỉ pump callback ROS input.

## Chạy live

### Board — node chờ sensor (robot / driver thật)

```bash
cd /home/theph/ws_ncs/gici_ros2_standard_interface
./scripts/ros2/launch_gici_live.sh board 1.1
# Terminal khác: publish GNSS/IMU/camera hoặc ros2 bag play ...
```

### Board — test với bag replay

```bash
./scripts/ros2/launch_gici_live.sh board 1.1 bag 1.0
# Sim time:
USE_SIM_TIME=1 ./scripts/ros2/launch_gici_live.sh board 1.1 bag 1.0
```

### UrbanNav live + RViz

```bash
./scripts/ros2/launch_gici_live.sh urbannav medium 1.0
```

### Launch trực tiếp

```bash
python3 scripts/ros2/render_gici_config.py --mode live --out /tmp/live.yaml \
  --dataset-dir /path/to/1.1 --output-dir /tmp/out --rtcm-start 2023.03.20
ros2 launch gici_ros2 gici_live.launch.py config:=/tmp/live.yaml
```

## Config live

| Profile | Template YAML |
|---------|---------------|
| GICI board | `config/ros_gici_board_live_rrr.yaml` |
| UrbanNav RRR | `config/ros_urbannav_live_rrr.yaml` |

Điều kiện bắt buộc trong YAML live:
- Không có `type: post-file` streamer cho sensor chính
- `replay.enable: false`
- `output_tags` gồm `str_solution_odometry`, `str_solution_path`, `str_solution_pose`

## QoS cho sensor thật

Live config dùng `best_effort` + `keep_last` cho IMU/camera (tránh queue vô hạn).
GNSS giữ `reliable` + `keep_all` mặc định (burst observation).

Override trong YAML:
```yaml
qos:
  reliability: best_effort
  history: keep_last
  depth: 500
```

## GNSS custom messages

Input GNSS vẫn cần `gici_ros2_msgs` (observations, ephemerides, …). Với receiver
thật, cần driver publish đúng format hoặc dùng streamer serial/TCP/NTRIP của GICI
(core, không qua ROS).

## So với OpenVINS

| | OpenVINS | GICI live ROS2 |
|---|---|---|
| Node | VIO node ROS-native | Monolithic `gici_ros2_main` + YAML graph |
| Config | ROS params | YAML file (+ `config_file` param) |
| GNSS | Không | Custom `gici_ros2_msgs` |
| Output | `/ov_msckf/odomimu` | `/gici/odom`, `/gici/path`, `/gici/pose` |

GICI không tách thành nhiều node ROS; thuật toán vẫn "sống" trong ROS2 qua
topic I/O và `ros2 launch`.
