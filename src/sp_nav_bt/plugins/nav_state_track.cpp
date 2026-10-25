#include "nav_state_track.hpp"

#include <cmath>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "tf2/exceptions.h"

namespace sp_nav_bt
{

NavStateTrack::NavStateTrack(const std::string & name, const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf)
{
  if (!config().blackboard->template get<rclcpp::Node::SharedPtr>("node", node_)) {
    throw std::runtime_error("NavStateTrack: rclcpp::Node::SharedPtr missing in blackboard");
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  // 从odom_topic订阅实测速度，hypot(vx,vy)得到与坐标旋转无关的合速度
  const auto odom_topic = getInput<std::string>("odom_topic").value();
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, rclcpp::QoS(10),
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      current_speed_.store(std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y));
    });
}

BT::PortsList NavStateTrack::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "goal_pose", "Target pose (default: read from blackboard key 'goal_pose')"),
    BT::InputPort<double>(
      "goal_tolerance", 0.2, "Distance threshold to consider goal reached (metres)"),
    // 到达需要同时满足位置和速度阈值，速度来源由odom_topic指定
    BT::InputPort<double>("goal_speed_tolerance", 0.06, "到达时的线速度阈值 m/s"),
    BT::InputPort<std::string>("odom_topic", "/Odometry", "机器人里程计话题"),
    BT::OutputPort<double>("remaining_distance", "Euclidean distance to goal (metres)"),
    BT::OutputPort<double>("elapsed_time",       "Time elapsed since task started (seconds)")
  };
}

BT::NodeStatus NavStateTrack::onStart()
{
  start_time_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(node_->get_logger(), "[NavStateTrack] Task started, monitoring navigation progress.");
  RCLCPP_INFO(node_->get_logger(), "[NavStateTrack] onStart() entered.");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavStateTrack::onRunning()
{
  geometry_msgs::msg::PoseStamped goal_pose;
  if (!getInput<geometry_msgs::msg::PoseStamped>("goal_pose", goal_pose)) {

    if (!config().blackboard->template get<geometry_msgs::msg::PoseStamped>("goal_pose", goal_pose)) {
      RCLCPP_WARN(node_->get_logger(), "[NavStateTrack] goal_pose not available yet, RUNNING.");
      return BT::NodeStatus::RUNNING;
    }
  }

  const double elapsed_sec =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();

  double current_x = 0.0;
  double current_y = 0.0;
  try {
    auto transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    current_x = transform.transform.translation.x;
    current_y = transform.transform.translation.y;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(),
      "[NavStateTrack] TF lookup failed: %s. Will retry.", ex.what());

    setOutput("elapsed_time", elapsed_sec);
    config().blackboard->set<double>("elapsed_time", elapsed_sec);
    return BT::NodeStatus::RUNNING;
  }

  const double dx = goal_pose.pose.position.x - current_x;
  const double dy = goal_pose.pose.position.y - current_y;
  const double remaining = std::hypot(dx, dy);

  setOutput("remaining_distance", remaining);
  setOutput("elapsed_time",       elapsed_sec);

  config().blackboard->set<double>("remaining_distance", remaining);
  config().blackboard->set<double>("elapsed_time",       elapsed_sec);

  RCLCPP_DEBUG(node_->get_logger(),
    "[NavStateTrack] remaining=%.3f m  elapsed=%.2f s  pos=(%.2f, %.2f)",
    remaining, elapsed_sec, current_x, current_y);

  double tolerance = 0.2;
  getInput<double>("goal_tolerance", tolerance);
  // 读取速度阈值和最新合速度，与剩余距离共同判断到达
  const double speed_tolerance = getInput<double>("goal_speed_tolerance").value();
  const double speed = current_speed_.load();

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
    "[NavStateTrack] goal=(%.2f, %.2f) current=(%.2f, %.2f) remaining=%.3f "
    "speed=%.3f",
    goal_pose.pose.position.x, goal_pose.pose.position.y,
    current_x, current_y,
    remaining, speed);

  if (remaining < tolerance && speed < speed_tolerance) {
    RCLCPP_INFO(node_->get_logger(),
      "[NavStateTrack] Goal reached! remaining=%.3f m, elapsed=%.2f s", remaining, elapsed_sec);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void NavStateTrack::onHalted()
{
  setStatus(BT::NodeStatus::IDLE);
  RCLCPP_INFO(node_->get_logger(), "[NavStateTrack] Halted externally.");
}

}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<sp_nav_bt::NavStateTrack>("nav_NavStateTrack");
}
