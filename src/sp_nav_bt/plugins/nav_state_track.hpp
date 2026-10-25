#ifndef SP_NAV_BT_PLUGINS_NAV_STATE_TRACK_HPP_
#define SP_NAV_BT_PLUGINS_NAV_STATE_TRACK_HPP_

#include <chrono>
#include <atomic>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_msg/action/navigate_to_pose.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace sp_nav_bt
{

class NavStateTrack : public BT::StatefulActionNode
{
public:
  NavStateTrack(const std::string & name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  // 订阅里程计得到合速度；current_speed_用原子变量传给行为树线程
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::atomic<double> current_speed_{0.0};
  // 保存当前导航 Action 句柄，用于向上层发送反馈
  std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_msg::action::NavigateToPose>> goal_handle_;

  std::chrono::steady_clock::time_point start_time_;
  // 记录上次反馈时间，将常规反馈频率限制在约 10 Hz
  std::chrono::steady_clock::time_point last_feedback_time_;
};

}

#endif
