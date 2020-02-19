#pragma once

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif

#include <carla_msgs/CarlaStatus.h>
#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <carla_ros_scenario_runner_types/CarlaScenarioList.h>
#include <carla_ros_scenario_runner_types/CarlaScenarioRunnerStatus.h>
#include <nav_msgs/Odometry.h>

class QString;
class QTimer;
class QLineEdit;
class QPushButton;
class QProgressBar;
class QCheckBox;
class QComboBox;
class QJsonDocument;

namespace rviz {

class ViewController;
class FramePositionTrackingViewController;
}
namespace rviz_carla_plugin
{

class DriveWidget;
class IndicatorWidget;

class CarlaControlPanel: public rviz::Panel
{
Q_OBJECT
public:
  CarlaControlPanel( QWidget* parent = 0 );

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

public Q_SLOTS:
  void setVel( float linear_velocity_, float angular_velocity_ );

  void setTopic( const QString& topic );

protected Q_SLOTS:
  void sendVel();

  void carlaStepOnce();
  void carlaTogglePlayPause();
  void overrideVehicleControl(int state);
  void executeScenario();

  void updateCameraPos();

protected:

  virtual void onInitialize() override;

  void scenarioRunnerStatusChanged(const carla_ros_scenario_runner_types::CarlaScenarioRunnerStatus::ConstPtr &msg);
  void carlaStatusChanged(const carla_msgs::CarlaStatus::ConstPtr& msg);
  void egoVehicleStatusChanged(const carla_msgs::CarlaEgoVehicleStatus::ConstPtr &msg);
  void egoVehicleOdometryChanged(const nav_msgs::Odometry::ConstPtr &msg);
  void carlaScenariosChanged(const carla_ros_scenario_runner_types::CarlaScenarioList::ConstPtr &msg);
  carla_msgs::CarlaStatus::ConstPtr mCarlaStatus{nullptr};

  DriveWidget* drive_widget_;

  QPushButton* mTriggerScenarioButton;
  QPushButton* mPlayPauseButton;
  QPushButton* mStepOnceButton;
  QProgressBar* mThrottleBar;
  QProgressBar* mBrakeBar;
  QProgressBar* mSteerBar;
  QLineEdit* mPosLabel;
  QLineEdit* mSpeedLabel;
  QLineEdit* mHeadingLabel;
  QCheckBox* mOverrideVehicleControl;
  QComboBox* mScenarioSelection;
  IndicatorWidget* mIndicatorWidget;

  ros::Publisher mTwistPublisher;
  ros::Publisher mCarlaControlPublisher;
  ros::Publisher mEgoVehicleControlManualOverridePublisher;
  ros::Subscriber mCarlaStatusSubscriber;
  ros::Subscriber mEgoVehicleStatusSubscriber;
  ros::Subscriber mEgoVehicleOdometrySubscriber;
  ros::ServiceClient mExecuteScenarioClient;
  ros::Subscriber mScenarioSubscriber;
  ros::Subscriber mScenarioRunnerStatusSubscriber;
  ros::Publisher mCameraPosePublisher;

  carla_ros_scenario_runner_types::CarlaScenarioList::ConstPtr mCarlaScenarios;

  ros::NodeHandle nh_;

  float linear_velocity_;
  float angular_velocity_;
  
  bool mVehicleControlManualOverride{false};
  rviz::FramePositionTrackingViewController *mViewController{nullptr};

  QTimer* mEventTimer;
  QString mCameraFrame;
};

} // end namespace rviz_carla_plugin
