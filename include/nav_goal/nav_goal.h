#ifndef NAV_GOAL_PANEL_H
#define NAV_GOAL_PANEL_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#endif

#include <QSlider>
#include <QDoubleSpinBox>

#include <geometry_msgs/PoseStamped.h>

class QLineEdit;

namespace nav_goal
{
class NavGoalPanel : public rviz::Panel
{
  int Z_VALUE_PRECISION = 1000; // 3 decimal precision

  Q_OBJECT
public:
  NavGoalPanel(QWidget* parent = 0);

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

public Q_SLOTS:

protected Q_SLOTS:

  void updateTopic();

  void updateSlider();

  void updateCurrentZValue();

  void updateMinZValue();

  void updateMaxZValue();

  void setTopic(const QString& new_in_topic, const QString& new_out_topic);

  void navGoal2DCallback(const geometry_msgs::PoseStamped::ConstPtr& goal);

protected:
  // One-line text editor for entering the outgoing ROS topic name.
  QLineEdit* nav_goal_2d_in_topic_editor_;
  QLineEdit* nav_goal_3d_out_topic_editor_;

  // The current name of the output topic.
  QString nav_goal_2d_in_topic_;
  QString nav_goal_3d_out_topic_;

  QDoubleSpinBox* min_z_value_;
  QDoubleSpinBox* max_z_value_;
  QDoubleSpinBox* current_z_value_;
  QSlider* z_slider_;

  // The ROS node handle.
  ros::NodeHandle nh_;

  ros::Subscriber nav_goal_2d_sub_;

  ros::Publisher nav_goal_3d_pub_;
};
}

#endif // NAV_GOAL_PANEL_H
