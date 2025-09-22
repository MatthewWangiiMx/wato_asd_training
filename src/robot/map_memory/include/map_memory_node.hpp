#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"


#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();
    void costmapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallBack(const nav_msgs::msg::Odometry::SharedPtr msg);
    void mapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void timerCallBack();
    void updateMap();

  private:
    robot::MapMemoryCore map_memory_;

    nav_msgs::msg::OccupancyGrid global_map_;
    nav_msgs::msg::OccupancyGrid latest_costmap_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    static constexpr uint32_t map_pub_rate = 1000;
    static constexpr double distance_update = 1.5;
    static constexpr double map_res = 0.1;
    static constexpr int map_width = 300;
    static constexpr int map_height = 300;

    double robot_x_, robot_y_;
    double prev_x_, prev_y_;
    double yaw_;

    bool update_map_ = false;
    bool costmap_updated_ = false;
};

#endif 
