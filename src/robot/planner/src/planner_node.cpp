#include "planner_node.hpp"
#include <queue>

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&PlannerNode::mapCallBack, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 10, std::bind(&PlannerNode::goalCallBack, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&PlannerNode::odomCallBack, this, std::placeholders::_1));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallBack, this));

  state_ = State::WAITING_FOR_GOAL;

  current_map_ = nav_msgs::msg::OccupancyGrid();
  robot_pose_ = geometry_msgs::msg::Pose();
  goal_ = geometry_msgs::msg::PointStamped();

}

void PlannerNode::mapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  if (state_ == State::REACHING_GOAL) {
    planPath();
  }
}

void PlannerNode::goalCallBack(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::REACHING_GOAL;
  planPath();
}

void PlannerNode::odomCallBack(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallBack() {
  if (state_ == State::REACHING_GOAL) {
    if (goalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal Reached!");
      state_ = State::WAITING_FOR_GOAL;
    } else {
      planPath();
    }
  }
}

bool PlannerNode::goalReached() {
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::hypot(dx, dy) < SETTLE_RADIUS;
}

void PlannerNode::planPath() {
  if (!goal_received_ || current_map_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Unable to plan path: No goal or map");
    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = "sim_world";
  std::vector<CellIndex> path_cells;

  int width = current_map_.info.width;
  int height = current_map_.info.height;
  double resolution = current_map_.info.resolution;

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> pq;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, double, CellIndexHash> f_score;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;


  double curr_x = robot_pose_.position.x;
  double curr_y = robot_pose_.position.y;

  int init_x = static_cast<int>((curr_x - current_map_.info.origin.position.x) / resolution);
  int init_y = static_cast<int>((curr_y - current_map_.info.origin.position.y) / resolution);

  int goal_x = static_cast<int>((goal_.point.x - current_map_.info.origin.position.x) / resolution);
  int goal_y = static_cast<int>((goal_.point.y - current_map_.info.origin.position.y) / resolution);;

  CellIndex init_idxy (init_x, init_y);
  CellIndex goal_idxy (goal_x, goal_y);

  pq.push(AStarNode(init_idxy, 0));
  f_score[init_idxy] = std::hypot(goal_x - init_x, goal_y - init_y);
  g_score[init_idxy] = 0; //haven't moved 

  if (init_x < 0 || init_x >= width || init_y < 0 || init_y >= height) {
    RCLCPP_WARN(this->get_logger(), "Robot position out of map bounds: grid(%d, %d), map size(%d, %d)", init_x, init_y, width, height);
    return;
  }

  if (goal_x < 0 || goal_x >= height || goal_y < 0 || goal_y >= height) {
    RCLCPP_WARN(this->get_logger(), "Goal position out of map bounds: grid(%d, %d), map size(%d, %d)", goal_x, goal_y, width, height);
    return;
  }

  while (!pq.empty()) {
    CellIndex idxy = pq.top().index;
    int x = idxy.x, y = idxy.y;
    pq.pop();

  if (idxy == goal_idxy) {
  reconstructPath(came_from, idxy, path_cells);
  break;
  }

  for (int i = -1; i <= 1; i++) {
    for (int j = -1; j <= 1; j++) {
      if (i == 0 && j == 0) continue;
      if (x + i < 0 || x + i >= width || y + j < 0 || y + j >= height) continue;

      CellIndex new_idxy (x + i, y + j);

      int cell_value = current_map_.data[(y + j) * width + (x + i)];

      double movement_cost = (i == 0 || j == 0) ? 1.0 : std::sqrt(2.0);
      double g = g_score[idxy] + movement_cost + cell_value * COST_WEIGHTING;

      if (g_score.find(new_idxy) == g_score.end() || g < g_score[new_idxy]) {
        g_score[new_idxy] = g;
        double h = std::hypot((x + i) - goal_x, (y + j) - goal_y);
        f_score[new_idxy] = g + h;
        came_from[new_idxy] = idxy;
        pq.push(AStarNode(new_idxy, f_score[new_idxy]));
      }
    }
  }
}

path.poses.clear();
for (auto& cell : path_cells) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header = path.header;

  double x = cell.x * current_map_.info.resolution + current_map_.info.origin.position.x;
  double y = cell.y * current_map_.info.resolution + current_map_.info.origin.position.y;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;

  path.poses.push_back(pose);
}
if (path_cells.empty()) {
  RCLCPP_WARN(this->get_logger(), "No path found to goal");
  return;
}

path_pub_->publish(path);

}

void PlannerNode::reconstructPath(const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from, CellIndex current, std::vector<CellIndex>& path) {
  path.clear(); 
  path.push_back(current); 
  while (came_from.find(current) != came_from.end()) {
    current = came_from.at(current);
    path.insert(path.begin(), current);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
