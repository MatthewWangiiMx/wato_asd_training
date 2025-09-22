#include "map_memory_node.hpp"
#include <cmath>

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, std::bind(&MapMemoryNode::costmapCallBack, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallBack, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(map_pub_rate), std::bind(&MapMemoryNode::timerCallBack, this));

  global_map_ = nav_msgs::msg::OccupancyGrid();
  latest_costmap_ = nav_msgs::msg::OccupancyGrid();

  global_map_.header.stamp = this->now();
  global_map_.header.frame_id = "sim_world";
  global_map_.info.resolution = map_res;
  global_map_.info.width = map_width;
  global_map_.info.height = map_height;
  global_map_.info.origin.position.x = -map_width * map_res / 2.0;
  global_map_.info.origin.position.y = -map_height * map_res / 2.0;
  global_map_.data.assign(map_width * map_height, 0);

  robot_x_ = 0.0;
  robot_y_ = 0.0;
  yaw_ = 0.0;
  prev_x_ = 0.0;
  prev_y_ = 0.0;

  updateMap();
  map_pub_ -> publish(global_map_);
}

void MapMemoryNode::costmapCallBack (const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  costmap_updated_ = true;
}

void MapMemoryNode::odomCallBack (const nav_msgs::msg::Odometry::SharedPtr msg) {
  double curr_x = msg->pose.pose.position.x;
  double curr_y = msg->pose.pose.position.y;

  double distance = std::hypot(curr_x - prev_x_, curr_y - prev_y_);
  if (distance < distance_update) return;

  robot_x_ = curr_x;
  robot_y_ = curr_y;

  auto q = msg->pose.pose.orientation;
  yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  update_map_ = true;
}

void MapMemoryNode::timerCallBack() {
  if (!update_map_ || !costmap_updated_) return;

  updateMap();
  global_map_.header.stamp = this->now();
  map_pub_ -> publish(global_map_);

  update_map_ = false;
  costmap_updated_ = false;
}

void MapMemoryNode::updateMap() {
  if (std::isnan(robot_x_) || std::isnan(robot_y_)) return;

  double l_res = latest_costmap_.info.resolution;
  int l_width = latest_costmap_.info.width;
  int l_height = latest_costmap_.info.height;
  auto& l_data = latest_costmap_.data;

  double g_res = global_map_.info.resolution;
  int g_width = global_map_.info.width;
  int g_height = global_map_.info.height;
  auto& g_data = global_map_.data;

  for (int x = 0; x < l_width; x++) {
    for (int y = 0; y < l_height; y++) {
      double l_x = (x - l_width / 2) * l_res;
      double l_y = (y - l_height / 2) * l_res;

      double g_x = robot_x_ + (l_x * std::cos(yaw_) - l_y * std::sin(yaw_));
      double g_y = robot_y_ + (l_x * std::sin(yaw_) + l_y * std::cos(yaw_));

      int index_g_x = static_cast<int>(std::round(g_x / g_res + g_width / 2));
      int index_g_y = static_cast<int>(std::round(g_y / g_res + g_height / 2));
      
      if (index_g_x < 0 || index_g_x >= g_width || index_g_y < 0 || index_g_y >= g_height) continue;

      int8_t& g_cost = g_data[index_g_y * g_width + index_g_x];
      g_cost = std::max(g_cost, l_data[y * l_width + x]);
    }
  }

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
