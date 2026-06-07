#include "tb3_rrt_rviz_plugin/rrt_panel.hpp"

#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rviz_common/display_context.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace tb3_rrt_rviz_plugin
{


static QDoubleSpinBox * makePoseSpin(
  QWidget * parent,
  double lo, double hi, double step,
  const char * suffix)
{
  auto * w = new QDoubleSpinBox(parent);
  w->setRange(lo, hi);
  w->setSingleStep(step);
  w->setDecimals(3);
  w->setSuffix(suffix);
  w->setFixedWidth(120);
  return w;
}

RRTPlannerPanel::RRTPlannerPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * root = new QVBoxLayout(this);
  root->setContentsMargins(8, 8, 8, 8);
  root->setSpacing(6);

  auto * start_group = new QGroupBox("Start Pose", this);
  auto * start_form  = new QFormLayout(start_group);

  start_x_   = makePoseSpin(this, -50.0, 50.0,  0.1,  " m");
  start_y_   = makePoseSpin(this, -50.0, 50.0,  0.1,  " m");
  start_yaw_ = makePoseSpin(this,  -3.1416,  3.1416, 0.05, " rad");

  start_form->addRow("X:",   start_x_);
  start_form->addRow("Y:",   start_y_);
  start_form->addRow("Yaw:", start_yaw_);

  auto * start_btn_row = new QHBoxLayout;
  auto * btn_pub_start  = new QPushButton("Publish Start", this);
  auto * btn_robot_pose = new QPushButton("Use Robot Pose", this);
  start_btn_row->addWidget(btn_pub_start);
  start_btn_row->addWidget(btn_robot_pose);
  start_form->addRow(start_btn_row);

  auto * goal_group = new QGroupBox("Goal Pose", this);
  auto * goal_form  = new QFormLayout(goal_group);

  goal_x_   = makePoseSpin(this, -50.0, 50.0,  0.1,  " m");
  goal_y_   = makePoseSpin(this, -50.0, 50.0,  0.1,  " m");
  goal_yaw_ = makePoseSpin(this,  -3.1416,  3.1416, 0.05, " rad");

  goal_form->addRow("X:",   goal_x_);
  goal_form->addRow("Y:",   goal_y_);
  goal_form->addRow("Yaw:", goal_yaw_);

  auto * goal_btn_row  = new QHBoxLayout;
  auto * btn_pub_goal  = new QPushButton("Publish Goal", this);
  goal_btn_row->addWidget(btn_pub_goal);
  goal_form->addRow(goal_btn_row);


  auto * btn_plan = new QPushButton("▶  Publish Both & Plan!", this);
  btn_plan->setStyleSheet(
    "QPushButton {"
    "  background-color: #2e7d32;"
    "  color: white;"
    "  font-weight: bold;"
    "  padding: 7px;"
    "  border-radius: 4px;"
    "}"
    "QPushButton:hover { background-color: #388e3c; }"
    "QPushButton:pressed { background-color: #1b5e20; }");


  status_label_ = new QLabel("Idle — waiting for map…", this);
  status_label_->setWordWrap(true);
  status_label_->setAlignment(Qt::AlignCenter);
  status_label_->setStyleSheet("color: #555; font-style: italic;");


  root->addWidget(start_group);
  root->addWidget(goal_group);
  root->addWidget(btn_plan);
  root->addWidget(status_label_);
  root->addStretch();

  setLayout(root);


  connect(btn_pub_start,  &QPushButton::clicked, this, &RRTPlannerPanel::onPublishStart);
  connect(btn_robot_pose, &QPushButton::clicked, this, &RRTPlannerPanel::onUseRobotPose);
  connect(btn_pub_goal,   &QPushButton::clicked, this, &RRTPlannerPanel::onPublishGoal);
  connect(btn_plan,       &QPushButton::clicked, this, &RRTPlannerPanel::onRunPlanner);
}


void RRTPlannerPanel::onInitialize()
{
  ros_node_ = getDisplayContext()
                ->getRosNodeAbstraction()
                .lock()
                ->get_raw_node();

  start_pub_ = ros_node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/start_pose", rclcpp::QoS(1));

  goal_pub_  = ros_node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", rclcpp::QoS(1));

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(ros_node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  setStatus("Initialised — set poses and click Plan!", true);
}


void RRTPlannerPanel::onPublishStart()
{
  if (!start_pub_) { return; }
  start_pub_->publish(makePose(start_x_->value(),
                               start_y_->value(),
                               start_yaw_->value()));
  setStatus(
    QString("Start published: (%1, %2, %3 rad)")
      .arg(start_x_->value(),   0, 'f', 2)
      .arg(start_y_->value(),   0, 'f', 2)
      .arg(start_yaw_->value(), 0, 'f', 2));
}

void RRTPlannerPanel::onPublishGoal()
{
  if (!goal_pub_) { return; }
  goal_pub_->publish(makePose(goal_x_->value(),
                              goal_y_->value(),
                              goal_yaw_->value()));
  setStatus(
    QString("Goal published: (%1, %2, %3 rad)")
      .arg(goal_x_->value(),   0, 'f', 2)
      .arg(goal_y_->value(),   0, 'f', 2)
      .arg(goal_yaw_->value(), 0, 'f', 2));
}

void RRTPlannerPanel::onUseRobotPose()
{
  if (!tf_buffer_) {
    setStatus("TF buffer not ready yet.", false);
    return;
  }

  try {
    auto tf = tf_buffer_->lookupTransform(
      "map", "base_footprint", tf2::TimePointZero);

    start_x_->setValue(tf.transform.translation.x);
    start_y_->setValue(tf.transform.translation.y);

    tf2::Quaternion q(
      tf.transform.rotation.x,
      tf.transform.rotation.y,
      tf.transform.rotation.z,
      tf.transform.rotation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    start_yaw_->setValue(yaw);

    setStatus(
      QString("Robot pose captured: (%1, %2, %3 rad)")
        .arg(start_x_->value(),   0, 'f', 2)
        .arg(start_y_->value(),   0, 'f', 2)
        .arg(start_yaw_->value(), 0, 'f', 2));

  } catch (const tf2::TransformException & ex) {
    setStatus(QString("TF error: %1").arg(ex.what()), false);
  }
}

void RRTPlannerPanel::onRunPlanner()
{
  onPublishStart();
  onPublishGoal();
  setStatus(
    QString("Planning: (%1,%2) → (%3,%4)")
      .arg(start_x_->value(), 0, 'f', 2)
      .arg(start_y_->value(), 0, 'f', 2)
      .arg(goal_x_->value(),  0, 'f', 2)
      .arg(goal_y_->value(),  0, 'f', 2));
}


geometry_msgs::msg::PoseStamped
RRTPlannerPanel::makePose(double x, double y, double yaw) const
{
  geometry_msgs::msg::PoseStamped ps;
  ps.header.stamp    = ros_node_ ? ros_node_->now() : rclcpp::Time(0);
  ps.header.frame_id = "map";
  ps.pose.position.x = x;
  ps.pose.position.y = y;
  ps.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  ps.pose.orientation.x = q.x();
  ps.pose.orientation.y = q.y();
  ps.pose.orientation.z = q.z();
  ps.pose.orientation.w = q.w();

  return ps;
}

void RRTPlannerPanel::setStatus(const QString & msg, bool ok)
{
  status_label_->setText(msg);
  status_label_->setStyleSheet(
    ok ? "color: #1b5e20; font-weight: bold;"
       : "color: #b71c1c; font-weight: bold;");
}

}  // namespace tb3_rrt_rviz_plugin

PLUGINLIB_EXPORT_CLASS(
  tb3_rrt_rviz_plugin::RRTPlannerPanel, rviz_common::Panel)
