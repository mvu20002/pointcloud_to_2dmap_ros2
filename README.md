# PointCloud to 2D Map ROS2

This ROS2 package converts point cloud data (PCD files) to 2D occupancy maps, compatible with ROS2 Humble.

## Original Repository

The original pointcloud_to_2dmap project provided a standalone C++ application for converting point clouds to 2D maps. This fork extends the functionality with a complete ROS2 wrapper.

## ROS2 Wrapper Implementation

**Implemented by**: Claude Sonnet 3.5  
**Conversion Features**:
- Complete ROS2 node architecture with proper parameter handling
- YAML configuration file support following ROS2 standards
- Occupancy grid publishing for real-time visualization
- Service interface for dynamic map generation
- Launch file support for easy deployment
- Full integration with ROS2 navigation stack

## Features

- Converts PCD files to 2D occupancy maps
- Generates PNG images and YAML files compatible with ROS2 navigation stack
- Publishes occupancy grids as ROS2 topics
- Configurable parameters via YAML files
- Service interface for on-demand map generation

## Dependencies

- ROS2 Humble
- PCL (Point Cloud Library)
- OpenCV
- Boost filesystem

## Building

```bash
cd ~/your_ws
colcon build --packages-select pointcloud_to_2dmap_ros2
source install/setup.bash
```

## Usage

### 1. Using Launch File with Config File

```bash
# Modify config/config.yaml or config/example_config.yaml with your parameters
ros2 launch pointcloud_to_2dmap_ros2 pointcloud_to_2dmap.launch.py
```

### 2. Running Node Directly with Parameters

```bash
ros2 run pointcloud_to_2dmap_ros2 pointcloud_to_2dmap_node \
  --ros-args \
  -p input_pcd:="/path/to/your/pointcloud.pcd" \
  -p dest_directory:="/path/to/output" \
  -p resolution:=0.05 \
  -p map_width:=2048 \
  -p map_height:=2048
```

### 3. Using Parameter File

```bash
ros2 run pointcloud_to_2dmap_ros2 pointcloud_to_2dmap_node \
  --ros-args --params-file src/pointcloud_to_2dmap_ros2/config/config.yaml
```

### 4. Service Interface

The node provides a service to generate maps on demand:

```bash
# Call the service to generate a map
ros2 service call /generate_map std_srvs/srv/Trigger
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `resolution` | double | 0.1 | Pixel resolution (meters per pixel) |
| `map_width` | int | 1024 | Map width in pixels |
| `map_height` | int | 1024 | Map height in pixels |
| `min_points_in_pix` | int | 2 | Minimum points in a pixel to be considered occupied |
| `max_points_in_pix` | int | 5 | Maximum points in a pixel for saturation |
| `min_height` | double | 0.5 | Minimum height of clipping range (meters) |
| `max_height` | double | 1.0 | Maximum height of clipping range (meters) |
| `input_pcd` | string | "" | Path to input PCD file |
| `dest_directory` | string | "" | Destination directory for output files |
| `frame_id` | string | "map" | Frame ID for the occupancy grid |
| `publish_occupancy_grid` | bool | true | Whether to publish occupancy grid topic |

## Topics

### Published Topics

- `/occupancy_grid` (nav_msgs/msg/OccupancyGrid): The generated occupancy grid

## Services

- `/generate_map` (std_srvs/srv/Trigger): Generate map from the configured PCD file

## Output Files

When `dest_directory` is specified, the node generates:
- `map.png`: The occupancy map as a PNG image
- `map.yaml`: Metadata file compatible with ROS2 navigation stack

## Configuration File Format

Create a YAML file with the following structure:

```yaml
pointcloud_to_2dmap_node:
  ros__parameters:
    resolution: 0.05
    map_width: 2048
    map_height: 2048
    min_points_in_pix: 3
    max_points_in_pix: 10
    min_height: 0.3
    max_height: 2.0
    input_pcd: "/path/to/your/pointcloud.pcd"
    dest_directory: "/path/to/output/directory"
    frame_id: "map"
    publish_occupancy_grid: true
```

## Example Workflow

1. Copy and modify the example config file:
   ```bash
   cp src/pointcloud_to_2dmap_ros2/config/example_config.yaml my_config.yaml
   # Edit my_config.yaml with your file paths and parameters
   ```

2. Run the node:
   ```bash
   ros2 run pointcloud_to_2dmap_ros2 pointcloud_to_2dmap_node \
     --ros-args --params-file my_config.yaml
   ```

3. View the occupancy grid in RViz2:
   ```bash
   rviz2
   # Add a Map display and subscribe to /occupancy_grid topic
   ```

## Migration from Original Code

This ROS2 version provides the same core functionality as the original standalone C++ application, with these enhancements:

- ROS2 parameter system instead of command-line arguments
- Occupancy grid publishing for real-time visualization
- Service interface for dynamic map generation
- Proper ROS2 package structure with launch files

![Screenshot_20200716_160239](https://user-images.githubusercontent.com/31344317/87637926-e7adfc00-c77d-11ea-8987-19dffe614fa5.png)
