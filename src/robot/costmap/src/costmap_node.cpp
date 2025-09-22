#include <chrono>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::lidarCallBack, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  //string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  //timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
}

void CostmapNode::lidarCallBack (const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  int width = 300, height = 300;
  double res = 0.1;
  int max_cost = 100;
  double inflation_radius = 1.0;

  std::vector<std::vector<int8_t>> costmap(width, std::vector<int8_t>(height, 0));

  for (size_t i = 0; i < msg->ranges.size(); i++) {
    if (msg->ranges[i] < msg->range_min || msg->ranges[i] > msg->range_max || std::isnan(msg->ranges[i])) continue;

    double angle = msg->angle_min + i * msg->angle_increment;
    double x = msg->ranges[i] * cos(angle);
    double y = msg->ranges[i] * sin(angle);

    int x_coord = static_cast<int>(x / res + width / 2);
    int y_coord = static_cast<int>(y / res + height / 2);

    if (x_coord < 0 || x_coord >= width || y_coord < 0 || y_coord >= height) continue;

    costmap[x_coord][y_coord] = max_cost;
    for (int dx = -inflation_radius / res; dx <= inflation_radius / res; dx++) {
      for (int dy = -inflation_radius / res; dy <= inflation_radius / res; dy++) {
        if (x_coord + dx < 0 || x_coord + dx >= width || y_coord + dy < 0 || y_coord + dy >= height) continue;

        double distance = sqrt(dx * dx + dy * dy) * res;
        if (distance > inflation_radius) continue;
        costmap[x_coord + dx][y_coord + dy] = std::max(static_cast<int>(costmap[x_coord + dx][y_coord + dy]), static_cast<int>((max_cost * (1 - std::min(1.0, distance / inflation_radius)))));
      }
    }
  }

nav_msgs::msg::OccupancyGrid occupancy_grid;
occupancy_grid.header.stamp = msg->header.stamp;
occupancy_grid.header.frame_id = msg->header.frame_id;
occupancy_grid.info.resolution = res;
occupancy_grid.info.width = width;
occupancy_grid.info.height = height;
occupancy_grid.info.origin.position.x = -width * res / 2;
occupancy_grid.info.origin.position.y = -height * res / 2;

occupancy_grid.data.resize(width * height);
for (int y = 0; y < height; y++) {
  for (int x = 0; x < width; x++) {
    occupancy_grid.data[y * width + x] = costmap[x][y];
  }
}

costmap_pub_->publish(occupancy_grid);

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}