#include "astar_planner.hpp"
#include <queue>
#include <limits>
#include <cmath>
#include <algorithm>
#include <rclcpp/exceptions.hpp>
#include <sstream>
#include <stdexcept>

namespace {

[[noreturn]] void sp_nav_param_error(const rclcpp::Node & node, const std::string & name)
{
  std::ostringstream oss;
  oss << "[参数缺失] 节点 '" << node.get_name() << "' 缺少参数 '" << name
      << "'，请在对应 yaml 的 ros__parameters 中填写。";
  throw std::runtime_error(oss.str());
}

template<typename T>
T require_param(const rclcpp::Node::SharedPtr & node, const std::string & name)
{
  try {
    if (node->has_parameter(name)) {
      return node->get_parameter(name).get_value<T>();
    }
    return node->declare_parameter<T>(name);
  } catch (const rclcpp::exceptions::UninitializedStaticallyTypedParameterException &) {
    sp_nav_param_error(*node, name);
  }
}

}

namespace sp_global_planner {

void AStarPlanner::configure(const rclcpp::Node::SharedPtr& node, const std::string& plugin_name)
{
  logger_ = node->get_logger();

  lethal_cost_  = require_param<int>(node, plugin_name + ".lethal_cost");
  cost_weight_  = require_param<double>(node, plugin_name + ".cost_weight");
  // 从 AStar 参数段读取原路径吸引权重、折角平滑权重、迭代次数和重采样间距
  smoothing_data_weight_ = require_param<double>(node, plugin_name + ".smoothing_data_weight");
  smoothing_smooth_weight_ = require_param<double>(node, plugin_name + ".smoothing_smooth_weight");
  smoothing_iterations_ = require_param<int>(node, plugin_name + ".smoothing_iterations");
  path_spacing_ = require_param<double>(node, plugin_name + ".path_spacing");

  RCLCPP_INFO(logger_, "AStarPlanner configured: lethal_cost=%d cost_weight=%.3f",
              lethal_cost_, cost_weight_);
}

void AStarPlanner::setMap(const nav_msgs::msg::OccupancyGrid& costmap)
{
  map_ = costmap;
}

bool AStarPlanner::isBlocked(int8_t c) const
{

  if (c < 0) return true;
  return c >= lethal_cost_;
}

double AStarPlanner::cellCostFactor(int8_t c) const
{

  double cc = std::max<int>(0, c);
  return 1.0 + cost_weight_ * (cc / 100.0);
}

bool AStarPlanner::isSegmentFree(
  const geometry_msgs::msg::Point& from,
  const geometry_msgs::msg::Point& to) const
{
  const auto& map = *map_;
  const double length = std::hypot(to.x - from.x, to.y - from.y);
  // 线段按四分之一栅格分辨率采样，并检查两端及中间采样点是否可通行。
  const int samples = std::max(1, static_cast<int>(std::ceil(length / (map.info.resolution * 0.25))));
  for (int i = 0; i <= samples; ++i) {
    const double ratio = static_cast<double>(i) / samples;
    GridIndex cell;
    if (!worldToGrid(map, from.x + ratio * (to.x - from.x),
                     from.y + ratio * (to.y - from.y), cell) ||
        isBlocked(map.data[toIndex(map, cell.x, cell.y)])) {
      return false;
    }
  }
  return true;
}

// 路径平滑
void AStarPlanner::smoothPath(nav_msgs::msg::Path& path) const
{
  const auto original = path.poses;
  // 固定起终点，用原路径约束偏移，并用相邻点拉平折角
  for (int iteration = 0; iteration < smoothing_iterations_; ++iteration) {
    for (size_t i = 1; i + 1 < path.poses.size(); ++i) {
      auto& current = path.poses[i].pose.position;
      const auto& before = path.poses[i - 1].pose.position;
      const auto& after = path.poses[i + 1].pose.position;
      const auto& anchor = original[i].pose.position;
      // candidate是待尝试的新位置；anchor拉回原路径，before/after减小折角
      auto candidate = current;
      candidate.x += smoothing_data_weight_ * (anchor.x - current.x) +
        smoothing_smooth_weight_ * (before.x + after.x - 2.0 * current.x);
      candidate.y += smoothing_data_weight_ * (anchor.y - current.y) +
        smoothing_smooth_weight_ * (before.y + after.y - 2.0 * current.y);
      // 代价地图检查，避免拉进墙壁
      if (isSegmentFree(before, candidate) && isSegmentFree(candidate, after)) {
        current = candidate;
      }
    }
  }

  std::vector<geometry_msgs::msg::PoseStamped> sampled{path.poses.front()};
  // traversed已走过的弧长，next_sample下一个等距采样位置
  double traversed = 0.0;
  double next_sample = path_spacing_;
  // 按弧长等距取点，使控制器的前视距离不受栅格方向影响
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto& from = path.poses[i - 1].pose.position;
    const auto& to = path.poses[i].pose.position;
    const double length = std::hypot(to.x - from.x, to.y - from.y);
    while (next_sample < traversed + length) {
      const double ratio = (next_sample - traversed) / length;
      auto pose = path.poses[i];
      pose.pose.position.x = from.x + ratio * (to.x - from.x);
      pose.pose.position.y = from.y + ratio * (to.y - from.y);
      sampled.push_back(pose);
      next_sample += path_spacing_;
    }
    traversed += length;
  }
  // 补回精确终点，避免采样间距使路径提前结束
  const auto& last = sampled.back().pose.position;
  const auto& goal = path.poses.back().pose.position;
  if (std::hypot(goal.x - last.x, goal.y - last.y) > 0.0) {
    sampled.push_back(path.poses.back());
  }
  path.poses = std::move(sampled);
  for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
    const auto& from = path.poses[i].pose.position;
    const auto& to = path.poses[i + 1].pose.position;
    // 用相邻采样点方向设置路径朝向，终点沿用最后一段朝向
    const double yaw = std::atan2(to.y - from.y, to.x - from.x);
    path.poses[i].pose.orientation.z = yaw_to_quat_z(yaw);
    path.poses[i].pose.orientation.w = yaw_to_quat_w(yaw);
  }
  if (path.poses.size() > 1) {
    path.poses.back().pose.orientation = path.poses[path.poses.size() - 2].pose.orientation;
  }
}

nav_msgs::msg::Path AStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped& start,
  const geometry_msgs::msg::PoseStamped& goal)
{
  nav_msgs::msg::Path path;
  if (!map_) {
    RCLCPP_ERROR(logger_, "No map received yet.");
    return path;
  }
  const auto& map = *map_;
  path.header = map.header;

  if (start.header.frame_id != map.header.frame_id || goal.header.frame_id != map.header.frame_id) {
    RCLCPP_WARN(logger_, "Frame mismatch: start=%s goal=%s map=%s",
      start.header.frame_id.c_str(), goal.header.frame_id.c_str(), map.header.frame_id.c_str());
  }

  GridIndex s, g;
  if (!worldToGrid(map, start.pose.position.x, start.pose.position.y, s)) {
    RCLCPP_ERROR(logger_, "Start out of map bounds.");
    return path;
  }
  if (!worldToGrid(map, goal.pose.position.x, goal.pose.position.y, g)) {
    RCLCPP_ERROR(logger_, "Goal out of map bounds.");
    return path;
  }

  const int W = static_cast<int>(map.info.width);
  const int H = static_cast<int>(map.info.height);
  const int N = W * H;

  auto idx = [&](int x, int y){ return y * W + x; };

  if (isBlocked(map.data[idx(s.x, s.y)])) {
    RCLCPP_ERROR(logger_, "Start is in blocked cell (cost=%d).", (int)map.data[idx(s.x, s.y)]);
    return path;
  }
  if (isBlocked(map.data[idx(g.x, g.y)])) {
    RCLCPP_ERROR(logger_, "Goal is in blocked cell (cost=%d).", (int)map.data[idx(g.x, g.y)]);
    return path;
  }

  struct Node {
    int i;
    double f;
    double g;
  };
  struct Cmp { bool operator()(const Node& a, const Node& b) const { return a.f > b.f; } };

  std::priority_queue<Node, std::vector<Node>, Cmp> open;
  std::vector<double> gscore(N, std::numeric_limits<double>::infinity());
  std::vector<int> parent(N, -1);
  std::vector<uint8_t> closed(N, 0);

  auto h = [&](int x, int y) {
    double dx = (x - g.x);
    double dy = (y - g.y);
    return std::sqrt(dx*dx + dy*dy);
  };

  int s_i = idx(s.x, s.y);
  int g_i = idx(g.x, g.y);

  gscore[s_i] = 0.0;
  open.push({s_i, h(s.x, s.y), 0.0});

  const int dxs[8] = {1,-1,0,0, 1,1,-1,-1};
  const int dys[8] = {0,0,1,-1, 1,-1,1,-1};

  bool found = false;

  while (!open.empty()) {
    Node cur = open.top();
    open.pop();

    if (closed[cur.i]) continue;
    closed[cur.i] = 1;

    if (cur.i == g_i) {
      found = true;
      break;
    }

    int cy = cur.i / W;
    int cx = cur.i - cy * W;

    for (int k = 0; k < 8; ++k) {
      int nx = cx + dxs[k];
      int ny = cy + dys[k];
      if (!inBounds(map, nx, ny)) continue;

      int ni = idx(nx, ny);
      if (closed[ni]) continue;

      int8_t c = map.data[ni];
      if (isBlocked(c)) continue;
      // 斜向移动时，保证两侧格子能通过，避免从墙角穿过去
      if (k >= 4 && (isBlocked(map.data[idx(nx, cy)]) ||
                     isBlocked(map.data[idx(cx, ny)]))) continue;

      double step = (k < 4) ? 1.0 : std::sqrt(2.0);

      double factor = cellCostFactor(c);
      double tentative = gscore[cur.i] + step * factor;

      if (tentative < gscore[ni]) {
        gscore[ni] = tentative;
        parent[ni] = cur.i;
        double f = tentative + h(nx, ny);
        open.push({ni, f, tentative});
      }
    }
  }

  if (!found) {
    RCLCPP_WARN(logger_, "A* failed to find a path.");
    return path;
  }

  std::vector<int> cells;
  int cur = g_i;
  while (cur != -1) {
    cells.push_back(cur);
    if (cur == s_i) break;
    cur = parent[cur];
  }
  if (cells.back() != s_i) {
    RCLCPP_WARN(logger_, "Path reconstruction failed.");
    return path;
  }
  std::reverse(cells.begin(), cells.end());

  path.poses.reserve(cells.size());
  for (int ci : cells) {
    int y = ci / W;
    int x = ci - y * W;

    double wx, wy;
    gridToWorld(map, x, y, wx, wy);

    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = wx;
    ps.pose.position.y = wy;
    ps.pose.position.z = 0.0;
    ps.pose.orientation.w = 1.0;
    path.poses.push_back(ps);
  }

  // 同一栅格内也保留起终点两个位置，表达最后一小段运动
  if (path.poses.size() == 1) {
    path.poses.push_back(path.poses.front());
  }
  // 使用实际起终点替换栅格中心，使规划终点与到达判据一致
  path.poses.front().pose.position = start.pose.position;
  path.poses.back().pose.position = goal.pose.position;
  smoothPath(path);

  return path;
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(sp_global_planner::AStarPlanner, sp_global_planner::GlobalPlannerPlugin)
