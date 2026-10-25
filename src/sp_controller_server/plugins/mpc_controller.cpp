#include "mpc_controller.hpp"

#include <cmath>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mpc_controller
{

void MpcController::configure(
  const rclcpp::Node::SharedPtr & node, const std::string & name,
  const std::shared_ptr<tf2_ros::Buffer> &)
{
  node_ = node;
  base_frame_id_ = node_->declare_parameter<std::string>(name + ".base_frame_id", "base_link");
  const auto parameter = [&](const std::string & key, double value) {
      return node_->declare_parameter<double>(name + "." + key, value);
    };
  // dt_ 控制周期
  dt_ = 1.0 / node_->get_parameter("control_frequency").as_double();
  // max_speed_、acceleration_、lateral_acceleration_ 用于安排参考轨迹的速度
  max_speed_ = parameter("max_speed", 1.6);
  acceleration_ = parameter("max_acceleration", 1.0);
  lateral_acceleration_ = parameter("lateral_acceleration", 0.8);
  reference_.configure(max_speed_, acceleration_, lateral_acceleration_);
  // command_limit_ 限制指令，actual_speed_limit_ 和 actual_acceleration_limit_ 限制预测运动
  command_limit_ = parameter("command_limit", 2.0);
  actual_speed_limit_ = parameter("actual_speed_limit", 2.0);
  actual_acceleration_limit_ = parameter("actual_acceleration_limit", 2.0);
  observer_gain_ = parameter("observer_gain", 1.0);
  // horizon_ 预测 80 步；move_steps_ 每4步共用一个指令，减少优化变量
  horizon_ = node_->declare_parameter<int>(name + ".prediction_steps", 80);
  move_steps_ = node_->declare_parameter<int>(name + ".move_steps", 4);
  iterations_ = node_->declare_parameter<int>(name + ".qp_iterations", 800);
  rho_ = parameter("qp_rho", 0.03);
  const double delay = parameter("command_delay", 0.1667);
  model_.configure(dt_, delay, parameter("filter_time_constant", 0.08),
    parameter("velocity_time_constant", 0.2923));
  // first_controlled_ 前的运动由旧指令决定；从新指令能生效的预测步开始约束运动
  first_controlled_ = static_cast<int>(std::ceil(delay / dt_));
  prediction_ = sp_controller_server::predictive::buildPrediction(model_, horizon_, move_steps_);
  moves_ = static_cast<int>(prediction_.Gp.cols());
  // state_x_、state_y_ 分别保存地图系两个方向的位置、速度、滤波速度和旧指令
  state_x_ = Eigen::VectorXd::Zero(model_.stateSize());
  state_y_ = Eigen::VectorXd::Zero(model_.stateSize());
  solution_ = Eigen::VectorXd::Zero(2 * moves_);
  position_weight_ = parameter("position_weight", 30.0);
  velocity_weight_ = parameter("velocity_weight", 2.0);
  input_weight_ = parameter("input_weight", 0.12);
  // 过程权重乘 dt_ 近似时间积分；末端额外权重强调预测末端的位置和速度
  weights_position_ = Eigen::VectorXd::Constant(horizon_, position_weight_ * dt_);
  weights_velocity_ = Eigen::VectorXd::Constant(horizon_, velocity_weight_ * dt_);
  weights_position_(horizon_ - 1) += parameter("terminal_position_weight", 12.0);
  weights_velocity_(horizon_ - 1) += parameter("terminal_velocity_weight", 2.0);
  // axis_hessian 将位置、速度误差写成二次项；输入项抑制过大的指令
  Eigen::MatrixXd axis_hessian = 2.0 * (
    prediction_.Gp.transpose() * weights_position_.asDiagonal() * prediction_.Gp +
    prediction_.Gv.transpose() * weights_velocity_.asDiagonal() * prediction_.Gv);
  axis_hessian.diagonal().array() += 2.0 * input_weight_ * dt_ * move_steps_;
  hessian_ = Eigen::MatrixXd::Zero(2 * moves_, 2 * moves_);
  hessian_.topLeftCorner(moves_, moves_) = axis_hessian;
  hessian_.bottomRightCorner(moves_, moves_) = axis_hessian;
  // solution_ 前 moves_项为x指令、后 moves_项为y指令；constraints_将两轴合并约束
  const int rows = 4 * moves_ + 8 * (horizon_ - first_controlled_);
  constraints_ = Eigen::MatrixXd::Zero(rows, 2 * moves_);
  lower_.resize(rows);
  upper_.resize(rows);
  int row = 0;
  for (int j = 0; j < moves_; ++j) {
    for (int d = 0; d < 4; ++d, ++row) {
      // 四个方向各设上下界，构成二维速度圆内的正八边形
      const double angle = M_PI / 8.0 + d * M_PI / 4.0;
      constraints_(row, j) = std::cos(angle);
      constraints_(row, moves_ + j) = std::sin(angle);
    }
  }
  for (int k = first_controlled_; k < horizon_; ++k) {
    for (int d = 0; d < 4; ++d) {
      const double angle = M_PI / 8.0 + d * M_PI / 4.0;
      const double c = std::cos(angle), s = std::sin(angle);
      constraints_.row(row).head(moves_) = c * prediction_.Gv.row(k);
      constraints_.row(row++).tail(moves_) = s * prediction_.Gv.row(k);
      constraints_.row(row).head(moves_) = c * prediction_.Ga.row(k);
      constraints_.row(row++).tail(moves_) = s * prediction_.Ga.row(k);
    }
  }
  // 固定的 hessian_ 和 constraints_ 在初始化时分解，供每个控制周期重复使用
  qp_.configure(hessian_, constraints_, rho_);
}

void MpcController::setPlan(const nav_msgs::msg::Path & path)
{
  // 把规划路径转换为按弧长查询的位置和速度参考
  reference_.setPlan(path);
}

geometry_msgs::msg::TwistStamped MpcController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = node_->now();
  command.header.frame_id = base_frame_id_;
  if (reference_.empty()) {
    return command;
  }
  // 用实测与预测速度之差修正滤波状态；旧指令队列继续保留
  state_x_(2) += observer_gain_ * (velocity.linear.x - state_x_(1));
  state_y_(2) += observer_gain_ * (velocity.linear.y - state_y_(1));
  // 状态下标 0 为位置、1 为实际速度、2 为滤波速度
  state_x_(0) = pose.pose.position.x;
  state_y_(0) = pose.pose.position.y;
  state_x_(1) = velocity.linear.x;
  state_y_(1) = velocity.linear.y;
  // preview[0] 是当前参考；preview[k+1] 对应第 k 行预测的下一时刻
  const auto preview = reference_.preview(
    state_x_(0), state_y_(0), state_x_(1), state_y_(1), dt_, horizon_);
  Eigen::VectorXd target_x(horizon_), target_y(horizon_);
  Eigen::VectorXd target_vx(horizon_), target_vy(horizon_);
  for (int k = 0; k < horizon_; ++k) {
    target_x(k) = preview[k + 1].x;
    target_y(k) = preview[k + 1].y;
    target_vx(k) = preview[k + 1].vx;
    target_vy(k) = preview[k + 1].vy;
  }
  // Fp/Fv/Fa 乘当前状态得到旧指令造成的运动；Gp/Gv/Ga 描述新指令的影响
  const Eigen::VectorXd px = prediction_.Fp * state_x_;
  const Eigen::VectorXd py = prediction_.Fp * state_y_;
  const Eigen::VectorXd vx = prediction_.Fv * state_x_;
  const Eigen::VectorXd vy = prediction_.Fv * state_y_;
  const Eigen::VectorXd ax = prediction_.Fa * state_x_;
  const Eigen::VectorXd ay = prediction_.Fa * state_y_;
  // gradient 是二次规划的一次项，由当前预测与目标位置、速度的偏差得到
  Eigen::VectorXd gradient(2 * moves_);
  gradient.head(moves_) = 2.0 * (
    prediction_.Gp.transpose() * weights_position_.asDiagonal() * (px - target_x) +
    prediction_.Gv.transpose() * weights_velocity_.asDiagonal() * (vx - target_vx));
  gradient.tail(moves_) = 2.0 * (
    prediction_.Gp.transpose() * weights_position_.asDiagonal() * (py - target_y) +
    prediction_.Gv.transpose() * weights_velocity_.asDiagonal() * (vy - target_vy));

  int row = 0;
  const double polygon_scale = std::cos(M_PI / 8.0);
  // 八边形约束同时限制两个方向的合速度、加速度和控制指令
  for (int j = 0; j < moves_; ++j) {
    for (int d = 0; d < 4; ++d, ++row) {
      lower_(row) = -command_limit_ * polygon_scale;
      upper_(row) = command_limit_ * polygon_scale;
    }
  }
  for (int k = first_controlled_; k < horizon_; ++k) {
    for (int d = 0; d < 4; ++d) {
      const double angle = M_PI / 8.0 + d * M_PI / 4.0;
      const double c = std::cos(angle), s = std::sin(angle);
      // 上下界减去当前状态贡献，使 constraints_ 只需乘待求的新指令序列
      const double speed = c * vx(k) + s * vy(k);
      lower_(row) = -actual_speed_limit_ * polygon_scale - speed;
      upper_(row++) = actual_speed_limit_ * polygon_scale - speed;
      const double acceleration = c * ax(k) + s * ay(k);
      lower_(row) = -actual_acceleration_limit_ * polygon_scale - acceleration;
      upper_(row++) = actual_acceleration_limit_ * polygon_scale - acceleration;
    }
  }
  // 用上一轮 solution_ 作为迭代初值，求解后只执行第一组 ux、uy
  solution_ = qp_.solve(gradient, lower_, upper_, solution_, iterations_);
  const double ux = solution_(0), uy = solution_(moves_);
  // 将本次指令加入时延队列，保留下一周期的预测状态
  state_x_ = model_.advance(state_x_, ux);
  state_y_ = model_.advance(state_y_, uy);
  // ux、uy在map系求得；按yaw旋转为base_link系速度交给底盘
  const double yaw = tf2::getYaw(pose.pose.orientation);
  command.twist.linear.x = std::cos(yaw) * ux + std::sin(yaw) * uy;
  command.twist.linear.y = -std::sin(yaw) * ux + std::cos(yaw) * uy;
  return command;
}

}

PLUGINLIB_EXPORT_CLASS(mpc_controller::MpcController, sp_controller_server::ControllerPlugin)
