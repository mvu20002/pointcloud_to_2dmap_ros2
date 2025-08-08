#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>

#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

class MapGenerater {
public:
  MapGenerater(double resolution, int map_width, int map_height, 
               int min_points_in_pix, int max_points_in_pix, 
               double min_height, double max_height)
  : resolution(resolution),
    m2pix(1.0 / resolution),
    map_width(map_width),
    map_height(map_height),
    min_points_in_pix(min_points_in_pix),
    max_points_in_pix(max_points_in_pix),
    min_height(min_height),
    max_height(max_height)
  {}

  cv::Mat generate(const pcl::PointCloud<pcl::PointXYZ>& cloud) const {
    cv::Mat map(map_height, map_width, CV_32SC1, cv::Scalar::all(0));

    for(const auto& point: cloud) {
      if(point.z < min_height || point.z > max_height) {
        continue;
      }

      int x = point.x * m2pix + map_width / 2;
      int y = -point.y * m2pix + map_width / 2;

      if(x < 0 || x >= map_width || y < 0 || y >= map_height) {
        continue;
      }

      map.at<int>(y, x) ++;
    }

    map -= min_points_in_pix;
    map.convertTo(map, CV_8UC1, - 255.0 / (max_points_in_pix - min_points_in_pix),  255);

    return map;
  }

  nav_msgs::msg::OccupancyGrid generateOccupancyGrid(const pcl::PointCloud<pcl::PointXYZ>& cloud, 
                                                     const std::string& frame_id) const {
    cv::Mat map_image = generate(cloud);
    
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = frame_id;
    grid.header.stamp = rclcpp::Clock().now();
    
    grid.info.resolution = resolution;
    grid.info.width = map_width;
    grid.info.height = map_height;
    grid.info.origin.position.x = -resolution * map_width / 2;
    grid.info.origin.position.y = -resolution * map_height / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    
    grid.data.resize(map_width * map_height);
    
    for(int y = 0; y < map_height; y++) {
      for(int x = 0; x < map_width; x++) {
        uint8_t pixel = map_image.at<uint8_t>(y, x);
        int8_t occupancy_value;
        
        if(pixel < 128) {
          occupancy_value = 100; // Occupied
        } else if(pixel > 200) {
          occupancy_value = 0;   // Free
        } else {
          occupancy_value = -1;  // Unknown
        }
        
        grid.data[y * map_width + x] = occupancy_value;
      }
    }
    
    return grid;
  }

public:
  const double resolution;    // meters per pixel
  const double m2pix;         // inverse resolution (pix/m)
  const int map_width;
  const int map_height;

  const int min_points_in_pix;
  const int max_points_in_pix;
  const double min_height;
  const double max_height;
};

class PointCloudTo2DMapNode : public rclcpp::Node {
public:
  PointCloudTo2DMapNode() : Node("pointcloud_to_2dmap_node") {
    // Declare parameters with default values
    this->declare_parameter("resolution", 0.1);
    this->declare_parameter("map_width", 1024);
    this->declare_parameter("map_height", 1024);
    this->declare_parameter("min_points_in_pix", 2);
    this->declare_parameter("max_points_in_pix", 5);
    this->declare_parameter("min_height", 0.5);
    this->declare_parameter("max_height", 1.0);
    this->declare_parameter("input_pcd", std::string(""));
    this->declare_parameter("dest_directory", std::string(""));
    this->declare_parameter("frame_id", std::string("map"));
    this->declare_parameter("publish_occupancy_grid", true);
    
    // Get parameters
    resolution_ = this->get_parameter("resolution").as_double();
    map_width_ = this->get_parameter("map_width").as_int();
    map_height_ = this->get_parameter("map_height").as_int();
    min_points_in_pix_ = this->get_parameter("min_points_in_pix").as_int();
    max_points_in_pix_ = this->get_parameter("max_points_in_pix").as_int();
    min_height_ = this->get_parameter("min_height").as_double();
    max_height_ = this->get_parameter("max_height").as_double();
    input_pcd_ = this->get_parameter("input_pcd").as_string();
    dest_directory_ = this->get_parameter("dest_directory").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    publish_occupancy_grid_ = this->get_parameter("publish_occupancy_grid").as_bool();
    
    // Create map generator
    map_generator_ = std::make_unique<MapGenerater>(
      resolution_, map_width_, map_height_, 
      min_points_in_pix_, max_points_in_pix_, 
      min_height_, max_height_
    );
    
    // Publishers
    if (publish_occupancy_grid_) {
      occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "occupancy_grid", 10);
    }
    
    // Service to generate map
    generate_map_service_ = this->create_service<std_srvs::srv::Trigger>(
      "generate_map", 
      std::bind(&PointCloudTo2DMapNode::generateMapCallback, this, 
                std::placeholders::_1, std::placeholders::_2)
    );
    
    // Log parameters
    RCLCPP_INFO(this->get_logger(), "PointCloud to 2D Map Node initialized");
    RCLCPP_INFO(this->get_logger(), "Input PCD: %s", input_pcd_.c_str());
    RCLCPP_INFO(this->get_logger(), "Destination directory: %s", dest_directory_.c_str());
    RCLCPP_INFO(this->get_logger(), "Resolution: %.3f", resolution_);
    RCLCPP_INFO(this->get_logger(), "Map size: %dx%d", map_width_, map_height_);
    
    // Auto-generate map if input file is provided
    if (!input_pcd_.empty()) {
      generateMapFromPCD();
    }
  }

private:
  void generateMapCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request; // Suppress unused parameter warning
    
    try {
      generateMapFromPCD();
      response->success = true;
      response->message = "Map generated successfully";
      RCLCPP_INFO(this->get_logger(), "Map generation service called successfully");
    } catch (const std::exception& e) {
      response->success = false;
      response->message = std::string("Failed to generate map: ") + e.what();
      RCLCPP_ERROR(this->get_logger(), "Map generation failed: %s", e.what());
    }
  }
  
  void generateMapFromPCD() {
    if (input_pcd_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Input PCD file not specified");
      return;
    }
    
    // Load point cloud
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if(pcl::io::loadPCDFile(input_pcd_, *cloud)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open the input cloud: %s", input_pcd_.c_str());
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Loaded point cloud with %zu points", cloud->size());
    
    // Generate map
    cv::Mat map = map_generator_->generate(*cloud);
    
    // Create destination directory if it doesn't exist
    if (!dest_directory_.empty()) {
      if(!boost::filesystem::exists(dest_directory_)) {
        boost::filesystem::create_directories(dest_directory_);
        RCLCPP_INFO(this->get_logger(), "Created directory: %s", dest_directory_.c_str());
      }
      
      // Save PNG image
      std::string png_path = dest_directory_ + "/map.png";
      cv::imwrite(png_path, map);
      RCLCPP_INFO(this->get_logger(), "Saved map image to: %s", png_path.c_str());
      
      // Save YAML file
      std::string yaml_path = dest_directory_ + "/map.yaml";
      std::ofstream ofs(yaml_path);
      ofs << "image: map.png" << std::endl;
      ofs << "resolution: " << map_generator_->resolution << std::endl;
      ofs << "origin: [" << -map_generator_->resolution * map_generator_->map_width / 2 << ", " 
          << -map_generator_->resolution * map_generator_->map_height / 2 << ", 0.0]" << std::endl;
      ofs << "occupied_thresh: 0.5" << std::endl;
      ofs << "free_thresh: 0.2" << std::endl;
      ofs << "negate: 0" << std::endl;
      ofs.close();
      RCLCPP_INFO(this->get_logger(), "Saved map YAML to: %s", yaml_path.c_str());
    }
    
    // Publish occupancy grid if enabled
    if (publish_occupancy_grid_ && occupancy_grid_pub_) {
      nav_msgs::msg::OccupancyGrid grid = map_generator_->generateOccupancyGrid(*cloud, frame_id_);
      occupancy_grid_pub_->publish(grid);
      RCLCPP_INFO(this->get_logger(), "Published occupancy grid");
    }
  }
  
  // Parameters
  double resolution_;
  int map_width_;
  int map_height_;
  int min_points_in_pix_;
  int max_points_in_pix_;
  double min_height_;
  double max_height_;
  std::string input_pcd_;
  std::string dest_directory_;
  std::string frame_id_;
  bool publish_occupancy_grid_;
  
  // ROS2 components
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr generate_map_service_;
  
  // Map generator
  std::unique_ptr<MapGenerater> map_generator_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudTo2DMapNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
