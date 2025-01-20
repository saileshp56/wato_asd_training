#include <chrono>
#include <memory>

#include "costmap_node.hpp" 
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  // subscribe to the /lider topic
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/laser", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
}
 
// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2! test123";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    const double origin_x = 0.0;
    const double origin_y = 0.0; 
    const double resolution = 0.1;
  // Step 1: Initialize costmap
    const int GRID_SIZE = 100;

    int grid[GRID_SIZE][GRID_SIZE] = {0};

    // Step 2: Convert LaserScan to grid and mark obstacles
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];

        if (range > scan->range_max || range < scan->range_min) {
          continue;
        }
        // Calculate grid coordinates
        int coordinate_x, coordinate_y;

        coordinate_x = range * cos(angle);
        coordinate_y = range * sin(angle);
        int grid_indice_x, grid_indice_y;

        int grid_index_x = static_cast<int>((coordinate_x - origin_x) / resolution);
        int grid_index_y = static_cast<int>((coordinate_y - origin_y) / resolution);


        if (grid_index_x < 100 && grid_index_x >= 0 && grid_index_y < 100 && grid_index_y >= 0) {
          grid[grid_index_x][grid_index_y] = 100;
        }
      }

    // Step 3: Inflate obstacles
    double inflation_radius = 1.0; // so 10 cells all around
    int inflation_cell_side = static_cast<int>(round(inflation_radius / 0.1));
    for (int x = 0; x < GRID_SIZE; ++x) {
      for (int y = 0; y < GRID_SIZE; ++y) {

        if (grid[x][y] == 100) {
          for (int i = -10; i < 0; ++i) {
            for (int j = -10; j < y + 10; ++j) {
              int neighboring_x = x + i;
              int neighboring_y = y + i;
              if (neighboring_x >= 0 && neighboring_x < GRID_SIZE && neighboring_y >= 0 && neighboring_y < GRID_SIZE) {
                double distance = sqrt((neighboring_x - x) * (neighboring_x - x) + (neighboring_y - y) * (neighboring_y - y)) / 10;
                int cost = static_cast<int>(100 * (1 - distance / inflation_radius));
                grid[neighboring_x][neighboring_y] = std::max(grid[neighboring_x][neighboring_y], cost);
              }
            }
          }
        }
      }
    }
    
    // Publish the Costmap
    nav_msgs::msg::OccupancyGrid occupancy_grid;
    occupancy_grid.header.stamp = this->now();
    occupancy_grid.header.frame_id = "map"; 
    occupancy_grid.info.width = 100;
    occupancy_grid.info.height = 100;
    occupancy_grid.info.resolution = 0.1;
    occupancy_grid.data.resize(100 * 100); 

    for (int x = 0; x < 100; ++x) {
        for (int y = 0; y < 100; ++y) {
            occupancy_grid.data[x + y * 100] = grid[x][y];
        }
    }

    grid_pub_->publish(occupancy_grid);

    }
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}