#pragma once
#include "sp_global_planner/global_planner_plugin.hpp"
#include "sp_global_planner/grid_utils.hpp"
#include <optional>
#include <vector>

namespace sp_global_planner {

class AStarPlanner : public GlobalPlannerPlugin
{
public:
  AStarPlanner() = default;
  ~AStarPlanner() override = default;

  void configure(const rclcpp::Node::SharedPtr& node, const std::string& plugin_name) override;
  void setMap(const nav_msgs::msg::OccupancyGrid& costmap) override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal) override;

private:
  std::optional<nav_msgs::msg::OccupancyGrid> map_;
  rclcpp::Logger logger_{rclcpp::get_logger("AStarPlanner")};

  int lethal_cost_{100};
  double cost_weight_{2.0};
  // 拉回原路径的权重，限制平滑后的偏移
  double smoothing_data_weight_{0.015};
  // 相邻点拉平折角的权重
  double smoothing_smooth_weight_{0.30};
  // 平滑更新次数
  int smoothing_iterations_{150};
  // 弧长重采样间隔
  double path_spacing_{0.05};

  bool isBlocked(int8_t c) const;
  double cellCostFactor(int8_t c) const;
  // 检查平滑点前后连线能否通过代价地图
  bool isSegmentFree(
    const geometry_msgs::msg::Point& from,
    const geometry_msgs::msg::Point& to) const;
  // 固定起终点平滑中间点，再按弧长重采样并设置朝向
  void smoothPath(nav_msgs::msg::Path& path) const;
};

}
