# lidar_mapping_bringup

Integrated launch package for a complete LiDAR-based mapping and localization system. This package orchestrates the execution of three key components:

1. **Livox LiDAR Driver** - Captures 3D point cloud data from Livox MID360
2. **FAST-LIO** - Real-time LiDAR-Inertial Odometry and Mapping
3. **OctoMap Server** - Builds 3D occupancy maps from point cloud data

## System Architecture

```
┌─────────────────────┐
│  Livox LiDAR Driver │
│  (msg_MID360)       │  Publishes: /livox/lidar
└──────────┬──────────┘
           │
           ▼
┌─────────────────────────────┐
│  FAST-LIO SLAM              │  Subscribes: /livox/lidar
│  (mapping.launch.py)        │  Publishes: /cloud_registered
└──────────┬──────────────────┘
           │
           ▼
┌──────────────────────────────┐
│  OctoMap Server              │  Subscribes: /cloud_registered
│  (octomap_mapping.launch.xml)│  Publishes: /octomap_binary
└──────────────────────────────┘
```

**Data Flow**: Point Cloud → SLAM Odometry → 3D Occupancy Map

## Installation

### Prerequisites
- ROS 2 (Humble, or later)
- `livox_ros_driver2` package (ROS2 version from source)
- `fast_lio` package (ROS2 compatible)
- `octomap_server` package

### Important: Single Workspace Setup

**All three component packages must be in the SAME ROS2 workspace and built together.**
This launch package includes the launch files from these packages, so they must all be discoverable via `ros2 pkg`.

### Build Instructions

```bash
# 1. Create or navigate to your ROS2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. Clone all required packages (if not already present)
# a) Livox driver (ROS2 version)
git clone https://github.com/Livox-SDK/livox_ros_driver2.git

# b) FAST-LIO
git clone --recursive https://github.com/Ericsii/FAST_LIO_ROS2.git

# c) OctoMap
git clone -b release https://github.com/taehun-kmu/OctoMap.git

# d) This launch package
git clone https://github.com/taehun-kmu/lidar-mapping-bringup.git lidar_mapping_bringup

# 3. Build all packages together
cd ~/ros2_ws
colcon build --packages-select livox_ros_driver2 fast_lio octomap_server lidar_mapping_bringup

# 4. Source the workspace
source install/setup.bash

# 5. Verify packages are discoverable
ros2 pkg list | grep -E "(livox|fast_lio|octomap|lidar_mapping)"
```

### Troubleshooting Build Issues

**"Package not found" error when running launch file:**
- Ensure you sourced the correct workspace: `source ~/ros2_ws/install/setup.bash`
- Verify all packages exist in your workspace: `ros2 pkg list | grep -E "(livox|fast_lio|octomap)"`
- Rebuild if needed: `colcon build --packages-select livox_ros_driver2 fast_lio octomap_server lidar_mapping_bringup`

## Usage

### Basic Usage

```bash
ros2 launch lidar_mapping_bringup system_bringup.launch.py
```

This will start all three components **simultaneously**:
1. Livox driver starts and publishes `/livox/lidar`
2. FAST-LIO starts and subscribes to `/livox/lidar` (ROS2 manages connection)
3. OctoMap server starts and subscribes to `/cloud_registered` (ROS2 manages connection)

ROS2 automatically manages topic connections, so each node automatically receives data from its upstream component without explicit sequencing.

### Launch Arguments

#### Common Arguments
| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `false` | Use simulation clock (Gazebo) |

#### Livox Driver Arguments
| Argument | Default | Description |
|----------|---------|-------------|
| `livox_config_path` | `<livox_pkg>/config` | Path to Livox config directory **Configurable** |
| `livox_frame_id` | `livox_frame` | TF frame ID for Livox data **Configurable** |
| `livox_bd_code` | `livox0000000001` | Board ID code for Livox device **Configurable** |

**Note:** Livox node is directly instantiated in Python, so all launch arguments take effect immediately.

#### FAST-LIO Arguments
| Argument | Default | Description |
|----------|---------|-------------|
| `fast_lio_config_path` | `<fast_lio_pkg>/config` | Path to FAST-LIO config directory |
| `fast_lio_config_file` | `mid360.yaml` | FAST-LIO configuration file name |
| `use_rviz` | `false` | Launch RViz for visualization |
| `rviz_config_path` | `<fast_lio_pkg>/rviz/fastlio.rviz` | Path to RViz config file |

#### OctoMap Arguments
| Argument | Default | Description |
|----------|---------|-------------|
| `octomap_resolution` | `0.05` | Voxel resolution in meters **Configurable** |
| `octomap_frame_id` | `camera_init` | Fixed map frame for OctoMap **Configurable** |
| `octomap_max_range` | `40.0` | Maximum integration range in meters **Configurable** |

**Note:** OctoMap node is directly instantiated in Python, so all launch arguments take effect immediately and override the defaults.

### Example Usage

#### Run with custom FAST-LIO config
```bash
ros2 launch lidar_mapping_bringup system_bringup.launch.py \
  fast_lio_config_file:=custom_config.yaml
```

#### Run with RViz visualization
```bash
ros2 launch lidar_mapping_bringup system_bringup.launch.py \
  use_rviz:=true
```

#### Run with simulation time
```bash
ros2 launch lidar_mapping_bringup system_bringup.launch.py \
  use_sim_time:=true
```

#### Customize OctoMap resolution
```bash
ros2 launch lidar_mapping_bringup system_bringup.launch.py \
  octomap_resolution:=0.1
```

#### Combine multiple arguments
```bash
ros2 launch lidar_mapping_bringup system_bringup.launch.py \
  use_rviz:=false \
  octomap_resolution:=0.05 \
  octomap_max_range:=20.0
```

## System Execution Flow

All three components start **simultaneously** via `IncludeLaunchDescription`:

1. **Livox Driver** (independent)
   - Initializes Livox LiDAR hardware
   - Publishes point cloud data on `/livox/lidar`

2. **FAST-LIO SLAM** (subscribes to Livox)
   - ROS2 automatically connects to `/livox/lidar` topic
   - Receives point cloud data as it becomes available
   - Performs SLAM computation
   - Publishes filtered/registered cloud on `/cloud_registered`

3. **OctoMap Server** (subscribes to FAST-LIO)
   - ROS2 automatically connects to `/cloud_registered` topic
   - Receives registered point cloud from FAST-LIO as it becomes available
   - Builds 3D occupancy map incrementally
   - Publishes map visualization on `/octomap_binary`

**Note**: ROS2's DDS middleware automatically manages publisher-subscriber connections and message delivery, eliminating the need for explicit event-based sequencing. This design is simpler, more robust, and follows ROS2 best practices.

### Simulation Time Synchronization

All three nodes (Livox, FAST-LIO, OctoMap) respect the `use_sim_time` parameter:
- When `use_sim_time:=true`, all nodes subscribe to `/clock` topic and use simulation time
- When `use_sim_time:=false` (default), all nodes use wall-clock time
- **Important**: For proper synchronization in simulation (Gazebo, Isaac Sim), ensure your simulator publishes to `/clock` topic
- All nodes must have `use_sim_time` set to the same value to maintain time coherence across the system

## Configuration Files

### Livox Configuration
- Location: `livox_ros_driver2/config/`
- Default: `MID360_config.json`
- Customize device settings, frame rates, output formats

### FAST-LIO Configuration
- Location: `fast_lio/config/`
- Default: `mid360.yaml`
- Contains LiDAR intrinsics, IMU settings, algorithm parameters

### OctoMap Configuration
- Parameters defined via launch arguments
- No external config file needed (XML launch file contains defaults)

### RViz Configuration
- Location: `lidar_mapping_bringup/rviz/`
- Default: `rviz.rviz` (integrated FAST-LIO + OctoMap visualization)
- **Displays**:
  - **TF**: Transform frames (camera_init ← body)
  - **Odometry**: FAST-LIO odometry trajectory
  - **Path**: SLAM trajectory history
  - **CloudRegistered**: Registered point cloud from FAST-LIO
  - **CloudMap**: Final SLAM map
  - **Map**: 2D projected occupancy grid from OctoMap (from `/projected_map` topic)
- **Fixed Frame**: `camera_init`
- Launch with `use_rviz:=true` to enable

## Troubleshooting

### Livox Driver Not Starting
```bash
# Check Livox connection
lsusb | grep Livox
# Verify permissions
sudo usermod -a -G dialout $USER
```

### FAST-LIO Not Starting
- Ensure Livox is publishing data: `ros2 topic echo /livox/lidar`
- Check config file path: `fast_lio_config_path`
- Verify IMU data if available

### OctoMap Not Generating
- Confirm FAST-LIO is publishing: `ros2 topic echo /cloud_registered`
- Check resolution is reasonable (0.05 - 0.10 meters)
- Monitor memory usage if running long

## Individual Package Launch

If needed, individual components can be launched separately:

```bash
# Livox only
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# FAST-LIO only
ros2 launch fast_lio mapping.launch.py

# OctoMap only
ros2 launch octomap_server octomap_mapping.launch.xml
```

## Performance Notes

- **CPU Usage**: High during SLAM computation (FAST-LIO)
- **Memory**: OctoMap memory grows with map size; adjust `octomap_resolution` to balance detail/memory
- **Latency**: Typical end-to-end latency 50-200ms
- **Recommended**: Run on Intel i7+ or equivalent ARM processor

## License

GPL-2.0

## References

- [FAST-LIO GitHub](https://github.com/Ericsii/FAST_LIO_ROS2)
- [Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2)
- [OctoMap](https://github.com/taehun-kmu/OctoMap/tree/release)
