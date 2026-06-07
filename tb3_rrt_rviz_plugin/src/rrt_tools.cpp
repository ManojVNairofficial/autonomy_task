#include "tb3_rrt_rviz_plugin/rrt_tools.hpp"
#include <sstream>
#include <tf2/LinearMath/Quaternion.h>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>

namespace tb3_rrt_rviz_plugin
{

/* -------------------------------------------------------------------------- */
/*                                 START POSE                                 */
/* -------------------------------------------------------------------------- */

SetRRTStartTool::SetRRTStartTool()
{
  shortcut_key_ = 's';
}

void SetRRTStartTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("Set RRT Start");

  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/start_pose", rclcpp::QoS(1));
}

void SetRRTStartTool::onPoseSet(double x, double y, double theta)
{
  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();

  geometry_msgs::msg::PoseStamped ps;
  ps.header.stamp    = node->now();
  ps.header.frame_id = "map";
  ps.pose.position.x = x;
  ps.pose.position.y = y;
  ps.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  ps.pose.orientation.x = q.x();
  ps.pose.orientation.y = q.y();
  ps.pose.orientation.z = q.z();
  ps.pose.orientation.w = q.w();

  RCLCPP_INFO(node->get_logger(),
    "RRT Start set: (%.3f, %.3f, %.3f rad)", x, y, theta);

  pub_->publish(ps);
}

/* -------------------------------------------------------------------------- */
/*                                  GOAL TOOL                                 */
/* -------------------------------------------------------------------------- */
SetRRTGoalTool::SetRRTGoalTool()
{
  shortcut_key_ = 'g';
}

void SetRRTGoalTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("Set RRT Goal");

  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", rclcpp::QoS(1));
}

void SetRRTGoalTool::onPoseSet(double x, double y, double theta)
{
  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();

  geometry_msgs::msg::PoseStamped ps;
  ps.header.stamp    = node->now();
  ps.header.frame_id = "map";
  ps.pose.position.x = x;
  ps.pose.position.y = y;
  ps.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  ps.pose.orientation.x = q.x();
  ps.pose.orientation.y = q.y();
  ps.pose.orientation.z = q.z();
  ps.pose.orientation.w = q.w();

  RCLCPP_INFO(node->get_logger(),
    "RRT Goal set: (%.3f, %.3f, %.3f rad) — RRT planner will start now.", x, y, theta);

  pub_->publish(ps);
}

}  // namespace tb3_rrt_rviz_plugin

PLUGINLIB_EXPORT_CLASS(tb3_rrt_rviz_plugin::SetRRTStartTool, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(tb3_rrt_rviz_plugin::SetRRTGoalTool,  rviz_common::Tool)
