#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_default_plugins/tools/pose/pose_tool.hpp>

namespace tb3_rrt_rviz_plugin
{

/**
 * SetRRTStartTool
 *
 * Works exactly like RViz2's "2D Pose Estimate":
 *   - Click on the map to place the start position
 *   - Hold and drag to set the heading angle
 *   - Releases a geometry_msgs/PoseStamped on /start_pose
 *
 * Shortcut key: S
 */
class SetRRTStartTool : public rviz_default_plugins::tools::PoseTool
{
  Q_OBJECT

public:
  SetRRTStartTool();
  ~SetRRTStartTool() override = default;

  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double theta) override;

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
};


/**
 * SetRRTGoalTool
 *
 * Works exactly like RViz2's "2D Goal Pose":
 *   - Click on the map to place the goal position
 *   - Hold and drag to set the heading angle
 *   - Releases a geometry_msgs/PoseStamped on /goal_pose
 *     which immediately triggers the RRT planner node
 *
 * Shortcut key: G
 */
class SetRRTGoalTool : public rviz_default_plugins::tools::PoseTool
{
  Q_OBJECT

public:
  SetRRTGoalTool();
  ~SetRRTGoalTool() override = default;

  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double theta) override;

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
};

}  // namespace tb3_rrt_rviz_plugin
