#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include <optional>
#include "control_core.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();


  private:
    robot::ControlCore control_;

    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr odom_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    static constexpr double ld_ = 0.5;
    static constexpr double goal_tolerance_ = 0.2;
    static constexpr double linear_vel_ = 1.0;
    static constexpr int dt = 100;
    static constexpr double linear_kp = 1.5;
    static constexpr double angular_kp = 0.8;

    void controlLoop();

    void pathCallBack(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallBack(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallBack();

    std::optional<geometry_msgs::msg::PoseStamped> findLookPoint();
    geometry_msgs::msg::Twist computeVel(const geometry_msgs::msg::PoseStamped& tgt);
    double computeDist(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b);
    double extractYaw(const geometry_msgs::msg::Quaternion& q);

};

#endif
