#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/path", 10, std::bind(&ControlNode::pathCallBack, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&ControlNode::odomCallBack, this, std::placeholders::_1));
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(dt), std::bind(&ControlNode::timerCallBack, this));

}

void ControlNode::pathCallBack(const nav_msgs::msg::Path::SharedPtr msg) {
  current_path_ = msg;
}

void ControlNode::odomCallBack(const nav_msgs::msg::Odometry::SharedPtr msg) {
  odom_ = msg;
}

void ControlNode::timerCallBack() {
  controlLoop();
}

void ControlNode::controlLoop() {
  if (!current_path_ || !odom_) {
    RCLCPP_WARN(this->get_logger(), "Path or odom not received yet");
    return;
  }

  double dist = computeDist(odom_->pose.pose.position, current_path_->poses.back().pose.position);
  if (dist < goal_tolerance_) {
    RCLCPP_INFO(this->get_logger(), "Goal reached, stopping robot");
    cmd_pub_->publish(geometry_msgs::msg::Twist());
    return;
  }

  auto look_point = findLookPoint();
  if (dist < ld_) {
    look_point = current_path_->poses.back();
  } else if (!look_point.has_value()) {
    RCLCPP_WARN(this->get_logger(), "No valid look point found");
    cmd_pub_->publish(geometry_msgs::msg::Twist());
    return;
  }

  auto cmd = computeVel(look_point.value());

  cmd_pub_->publish(cmd);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookPoint() {
  for (auto& pose : current_path_->poses) {
    double distance = computeDist(odom_->pose.pose.position, pose.pose.position);
    if (distance >= ld_) {
      return pose;
    }
  }
  return std::nullopt;
}

geometry_msgs::msg::Twist ControlNode::computeVel(const geometry_msgs::msg::PoseStamped& tgt) {
  double yaw = extractYaw(odom_->pose.pose.orientation);
  double x = tgt.pose.position.x - odom_->pose.pose.position.x;
  double y = tgt.pose.position.y - odom_->pose.pose.position.y;
  double tgt_angle = std::atan2(y, x);
  double angle_err = tgt_angle - yaw;

  while (angle_err > M_PI) angle_err -= 2* M_PI;
  while (angle_err < -M_PI) angle_err += 2* M_PI;

  double distance = computeDist(odom_->pose.pose.position, current_path_->poses.back().pose.position);
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = std::min(linear_vel_, linear_kp * distance);
  cmd.angular.z = tgt == current_path_->poses.back() ? 0.0 : angular_kp * angle_err;
  return cmd;
}

double ControlNode::computeDist(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
