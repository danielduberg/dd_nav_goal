#include <nav_goal/nav_goal.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QSlider>

namespace nav_goal
{
NavGoalPanel::NavGoalPanel(QWidget* parent) : rviz::Panel(parent)
{
  QGridLayout* layout = new QGridLayout();
  layout->addWidget(new QLabel("Input 2D Nav Goal Topic:"), 0, 0, 1, 2);
  nav_goal_2d_in_topic_editor_ = new QLineEdit;
  layout->addWidget(nav_goal_2d_in_topic_editor_, 0, 2, 1, 2);
  layout->addWidget(new QLabel("Output 3D Nav Goal Topic:"), 1, 0, 1, 2);
  nav_goal_3d_out_topic_editor_ = new QLineEdit;
  layout->addWidget(nav_goal_3d_out_topic_editor_, 1, 2, 1, 2);
  z_slider_ = new QSlider(Qt::Vertical);
  z_slider_->setMinimum(-10 * Z_VALUE_PRECISION);
  z_slider_->setMaximum(10 * Z_VALUE_PRECISION);
  z_slider_->setTickPosition(QSlider::NoTicks);
  layout->addWidget(new QLabel("Max altitude:"), 2, 0);
  max_z_value_ = new QDoubleSpinBox;
  max_z_value_->setMinimum(-std::numeric_limits<double>::infinity());
  max_z_value_->setMaximum(std::numeric_limits<double>::infinity());
  max_z_value_->setSingleStep(0.5);
  layout->addWidget(max_z_value_, 2, 2);
  layout->addWidget(z_slider_, 3, 2, 5, 1);
  layout->addWidget(new QLabel("Current altitude:"), 5, 0);
  current_z_value_ = new QDoubleSpinBox;
  current_z_value_->setMinimum(-std::numeric_limits<double>::infinity());
  current_z_value_->setMaximum(std::numeric_limits<double>::infinity());
  current_z_value_->setSingleStep(0.01);
  current_z_value_->setDecimals(3);
  current_z_value_->setAccelerated(true);
  layout->addWidget(current_z_value_, 5, 1);
  layout->addWidget(new QLabel("Min altitude:"), 8, 0);
  min_z_value_ = new QDoubleSpinBox;
  min_z_value_->setMinimum(-std::numeric_limits<double>::infinity());
  min_z_value_->setMaximum(std::numeric_limits<double>::infinity());
  min_z_value_->setSingleStep(0.5);
  layout->addWidget(min_z_value_, 8, 2);
  setLayout(layout);

  // Next we make signal/slot connections.
  connect(nav_goal_2d_in_topic_editor_, SIGNAL(editingFinished()), this,
          SLOT(updateTopic()));
  connect(nav_goal_3d_out_topic_editor_, SIGNAL(editingFinished()), this,
          SLOT(updateTopic()));
  connect(z_slider_, SIGNAL(valueChanged(int)), this, SLOT(updateSlider()));
  connect(current_z_value_, SIGNAL(valueChanged(double)), this, SLOT(updateCurrentZValue()));
  connect(min_z_value_, SIGNAL(valueChanged(double)), this, SLOT(updateMinZValue()));
  connect(max_z_value_, SIGNAL(valueChanged(double)), this, SLOT(updateMaxZValue()));
}

void NavGoalPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
  QString topic_in;
  if (config.mapGetString("nav_goal_2d_in_topic", &topic_in))
  {
    nav_goal_2d_in_topic_editor_->setText(topic_in);
    updateTopic();
  }

  QString topic_out;
  if (config.mapGetString("nav_goal_3d_out_topic", &topic_out))
  {
    nav_goal_3d_out_topic_editor_->setText(topic_out);
    updateTopic();
  }

  int slider_value;
  if (config.mapGetInt("z_slider_value", &slider_value))
  {
    z_slider_->setValue(slider_value);
  }

  float slider_min;
  if (config.mapGetFloat("z_slider_min", &slider_min))
  {
    min_z_value_->setValue(slider_min);
  }

  float slider_max;
  if (config.mapGetFloat("z_slider_max", &slider_max))
  {
    max_z_value_->setValue(slider_max);
  }
}

void NavGoalPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
  config.mapSetValue("nav_goal_2d_in_topic", nav_goal_2d_in_topic_);
  config.mapSetValue("nav_goal_3d_out_topic", nav_goal_3d_out_topic_);
  config.mapSetValue("z_slider_value", z_slider_->value());
  config.mapSetValue("z_slider_min", z_slider_->minimum());
  config.mapSetValue("z_slider_max", z_slider_->maximum());
}

void NavGoalPanel::updateTopic()
{
  setTopic(nav_goal_2d_in_topic_editor_->text(),
           nav_goal_3d_out_topic_editor_->text());
}

void NavGoalPanel::setTopic(const QString& new_in_topic,
                            const QString& new_out_topic)
{
  if (new_in_topic != nav_goal_2d_in_topic_ ||
      new_out_topic != nav_goal_3d_out_topic_)
  {
    nav_goal_2d_in_topic_ = new_in_topic;
    nav_goal_3d_out_topic_ = new_out_topic;
    if (nav_goal_3d_out_topic_ == "" || nav_goal_2d_in_topic_ == "")
    {
      nav_goal_2d_sub_.shutdown();
      nav_goal_3d_pub_.shutdown();
    }
    else
    {
      nav_goal_2d_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
          nav_goal_2d_in_topic_.toStdString(), 1,
          &NavGoalPanel::navGoal2DCallback, this);
      nav_goal_3d_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
          nav_goal_3d_out_topic_.toStdString(), 1);
    }
    // rviz::Panel defines the configChanged() signal.  Emitting it
    // tells RViz that something in this panel has changed that will
    // affect a saved config file.  Ultimately this signal can cause
    // QWidget::setWindowModified(true) to be called on the top-level
    // rviz::VisualizationFrame, which causes a little asterisk ("*")
    // to show in the window's title bar indicating unsaved changes.
    Q_EMIT configChanged();
  }

  // TODO: Disable slider?
}

void NavGoalPanel::navGoal2DCallback(
    const geometry_msgs::PoseStamped::ConstPtr& goal)
{
  geometry_msgs::PoseStamped new_goal = *goal;
  new_goal.pose.position.z = current_z_value_->value();
  nav_goal_3d_pub_.publish(new_goal);
}

void NavGoalPanel::updateSlider()
{
  current_z_value_->setValue(((double) z_slider_->value()) / Z_VALUE_PRECISION);
}

void NavGoalPanel::updateCurrentZValue()
{
  z_slider_->setValue(current_z_value_->value() * Z_VALUE_PRECISION);
}

void NavGoalPanel::updateMinZValue()
{
  z_slider_->setMinimum(min_z_value_->value() * Z_VALUE_PRECISION);
}

void NavGoalPanel::updateMaxZValue()
{
  z_slider_->setMaximum(max_z_value_->value() * Z_VALUE_PRECISION);
}
}

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(nav_goal::NavGoalPanel, rviz::Panel)
