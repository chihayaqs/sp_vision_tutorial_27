#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <nav_msgs/msg/path.hpp>

namespace sp_controller_server
{

// x、y 是地图系位置，vx、vy是同一坐标系下的参考速度
struct ReferencePoint
{
  double x;
  double y;
  double vx;
  double vy;
};

class TrajectoryReference
{
public:
  void configure(double max_speed, double acceleration, double lateral_acceleration)
  {
    max_speed_ = max_speed;
    acceleration_ = acceleration;
    lateral_acceleration_ = lateral_acceleration;
  }

  void setPlan(const nav_msgs::msg::Path & path)
  {
    points_.clear();
    distance_.clear();
    for (const auto & pose : path.poses) {
      const auto & p = pose.pose.position;
      if (!points_.empty() && std::hypot(p.x - points_.back().x, p.y - points_.back().y) < 1e-6) {
        continue;
      }
      // distance_ 累计路径弧长；重复点被跳过，避免产生零长度线段
      distance_.push_back(points_.empty() ? 0.0 : distance_.back() +
        std::hypot(p.x - points_.back().x, p.y - points_.back().y));
      points_.push_back({p.x, p.y, 0.0, 0.0});
    }
    speed_.assign(points_.size(), max_speed_);
    if (points_.empty()) {
      return;
    }
    // 按曲率限制转弯速度，再从终点倒推制动距离
    for (size_t i = 1; i + 1 < points_.size(); ++i) {
      const auto a = position(std::max(0.0, distance_[i] - 0.18));
      const auto b = points_[i];
      const auto c = position(std::min(distance_.back(), distance_[i] + 0.18));
      const double ab = std::hypot(b.x - a.x, b.y - a.y);
      const double bc = std::hypot(c.x - b.x, c.y - b.y);
      const double ac = std::hypot(c.x - a.x, c.y - a.y);
      const double cross = std::abs((b.x - a.x) * (c.y - a.y) -
        (b.y - a.y) * (c.x - a.x));
      // 用前后各0.18米的三个点估计曲率，按 v²×曲率不超过横向加速度安排转弯速度
      const double curvature = 2.0 * cross / (ab * bc * ac);
      if (curvature > 1e-6) {
        speed_[i] = std::min(max_speed_, std::sqrt(lateral_acceleration_ / curvature));
      }
    }
    // 终点速度为零；倒推v²+2aΔs得到能在终点停下的速度上界
    speed_.back() = 0.0;
    for (size_t i = speed_.size() - 1; i > 0; --i) {
      speed_[i - 1] = std::min(speed_[i - 1], std::sqrt(speed_[i] * speed_[i] +
        2.0 * acceleration_ * (distance_[i] - distance_[i - 1])));
    }
    // 再沿路径正向限制加速，使相邻参考速度满足切向加速度要求
    for (size_t i = 1; i < speed_.size(); ++i) {
      speed_[i] = std::min(speed_[i], std::sqrt(speed_[i - 1] * speed_[i - 1] +
        2.0 * acceleration_ * (distance_[i] - distance_[i - 1])));
    }
  }

  bool empty() const {return points_.empty();}

  std::vector<ReferencePoint> preview(
    double x, double y, double vx, double vy, double dt, size_t count) const
  {
    // s 为机器人投影到路径后的弧长，v 为实测速度沿路径切线的分量
    double s = project(x, y);
    auto tangent = direction(s);
    double v = std::max(0.0, vx * tangent.first + vy * tangent.second);
    std::vector<ReferencePoint> result;
    result.reserve(count + 1);
    for (size_t i = 0; i <= count; ++i) {
      auto p = position(s);
      tangent = direction(s);
      const double target = speedAt(s);
      // 每周期速度变化不超过acceleration_×dt，避免参考速度突然跳变
      const double next_v = std::clamp(target, v - acceleration_ * dt, v + acceleration_ * dt);
      p.vx = next_v * tangent.first;
      p.vy = next_v * tangent.second;
      if (s >= distance_.back()) {
        p.vx = 0.0;
        p.vy = 0.0;
      }
      result.push_back(p);
      // 用本步和下一步速度的平均值推进弧长，推进到终点后保持位置
      s = std::min(distance_.back(), s + 0.5 * (v + next_v) * dt);
      v = next_v;
    }
    return result;
  }

private:
  // 按累计弧长查找所在的路径线段
  size_t segment(double s) const
  {
    return std::min(static_cast<size_t>(std::upper_bound(distance_.begin(), distance_.end(), s) -
      distance_.begin() - 1), points_.size() - 2);
  }

  // 在线段上按弧长比例插值位置
  ReferencePoint position(double s) const
  {
    if (points_.size() == 1) {
      return points_.front();
    }
    const size_t i = segment(s);
    const double t = (s - distance_[i]) / (distance_[i + 1] - distance_[i]);
    return {points_[i].x + t * (points_[i + 1].x - points_[i].x),
      points_[i].y + t * (points_[i + 1].y - points_[i].y), 0.0, 0.0};
  }

  // 用前后各 0.04 米的弦方向估计单位切线
  std::pair<double, double> direction(double s) const
  {
    if (points_.size() == 1) {
      return {0.0, 0.0};
    }
    const auto a = position(std::max(0.0, s - 0.04));
    const auto b = position(std::min(distance_.back(), s + 0.04));
    const double length = std::hypot(b.x - a.x, b.y - a.y);
    return {(b.x - a.x) / length, (b.y - a.y) / length};
  }

  // 对速度平方插值，与线段上恒定加速度的距离关系一致
  double speedAt(double s) const
  {
    if (points_.size() == 1) {
      return 0.0;
    }
    const size_t i = segment(s);
    const double t = (s - distance_[i]) / (distance_[i + 1] - distance_[i]);
    return std::sqrt((1.0 - t) * speed_[i] * speed_[i] + t * speed_[i + 1] * speed_[i + 1]);
  }

  // 每次选择离实测位置最近的路径段，偏离路径后仍能向最近的安全位置修正。
  double project(double x, double y) const
  {
    double best = std::hypot(x - points_.front().x, y - points_.front().y);
    double result = 0.0;
    for (size_t i = 0; i + 1 < points_.size(); ++i) {
      const double dx = points_[i + 1].x - points_[i].x;
      const double dy = points_[i + 1].y - points_[i].y;
      const double length = distance_[i + 1] - distance_[i];
      const double t = std::clamp(((x - points_[i].x) * dx + (y - points_[i].y) * dy) /
        (length * length), 0.0, 1.0);
      const double error = std::hypot(x - points_[i].x - t * dx, y - points_[i].y - t * dy);
      if (error < best) {
        best = error;
        result = distance_[i] + t * length;
      }
    }
    return result;
  }

private:
  // points_ 存位置，distance_ 存累计米数，speed_ 存各点参考速度上界
  std::vector<ReferencePoint> points_;
  std::vector<double> distance_;
  std::vector<double> speed_;
  double max_speed_;
  double acceleration_;
  double lateral_acceleration_;
};

}
