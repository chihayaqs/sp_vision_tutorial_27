#pragma once

#include "sp_controller_server/controller_plugin.hpp"
#include "trajectory_reference.hpp"
#include "predictive_model.hpp"
#include "constrained_qp.hpp"

namespace mpc_controller
{

class MpcController : public sp_controller_server::ControllerPlugin
{
public:
  void configure(
    const rclcpp::Node::SharedPtr & node, const std::string & plugin_name,
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer) override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity) override;

private:
  rclcpp::Node::SharedPtr node_;
  std::string base_frame_id_;
  // reference_ 生成参考；model_ 描述延迟运动；prediction_ 展开未来；qp_ 求解指令。
  sp_controller_server::TrajectoryReference reference_;
  sp_controller_server::predictive::DelayedVelocityModel model_;
  sp_controller_server::predictive::Prediction prediction_;
  sp_controller_server::predictive::ConstrainedQp qp_;
  // state_x_/state_y_ 是模型状态，solution_ 是两个方向的未来指令序列。
  Eigen::VectorXd state_x_, state_y_, solution_;
  // hessian_ 表示二次代价，constraints_ 与 lower_/upper_ 表示线性约束。
  Eigen::MatrixXd hessian_, constraints_;
  Eigen::VectorXd lower_, upper_, weights_position_, weights_velocity_;
  // 参数含义和单位对应 nav_params.yaml 的 MpcController 段。
  double dt_, command_limit_, max_speed_, acceleration_, lateral_acceleration_;
  double actual_speed_limit_, actual_acceleration_limit_;
  double observer_gain_, position_weight_, velocity_weight_, input_weight_, rho_;
  // horizon_ 为预测步数，moves_ 为每轴指令数，first_controlled_ 为新指令首个生效输出下标。
  int horizon_, move_steps_, moves_, first_controlled_, iterations_;
};

}
