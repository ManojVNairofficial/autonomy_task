#pragma once

#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QWidget>

#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace tb3_rrt_rviz_plugin
{

class RRTPlannerPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit RRTPlannerPanel(QWidget * parent = nullptr);
  ~RRTPlannerPanel() override = default;

  void onInitialize() override;

private Q_SLOTS:
  void onPublishStart();
  void onPublishGoal();
  void onUseRobotPose();
  void onRunPlanner();

private:
  geometry_msgs::msg::PoseStamped makePose(double x, double y, double yaw) const;
  void setStatus(const QString & msg, bool ok = true);

  // ---- Start widgets ----
  QDoubleSpinBox * start_x_{nullptr};
  QDoubleSpinBox * start_y_{nullptr};
  QDoubleSpinBox * start_yaw_{nullptr};

  // ---- Goal widgets ----
  QDoubleSpinBox * goal_x_{nullptr};
  QDoubleSpinBox * goal_y_{nullptr};
  QDoubleSpinBox * goal_yaw_{nullptr};

  // ---- Status bar ----
  QLabel * status_label_{nullptr};

  // ---- ROS interfaces  ----
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr start_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace tb3_rrt_rviz_plugin
