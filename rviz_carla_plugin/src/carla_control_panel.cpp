#include <stdio.h>

#include <QPainter>
#include <QIcon>
#include <QPixmap>
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QProgressBar>
#include <QFormLayout>
#include <QCheckBox>
#include <QComboBox>
#include <QJsonDocument>
#include <QDebug>

#include <carla_ros_scenario_runner_types/ExecuteScenario.h>
#include <carla_msgs/CarlaControl.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>

#include "drive_widget.h"
#include "indicator_widget.h"
#include "carla_control_panel.h"
#include <rviz/visualization_manager.h>
#include <rviz/view_manager.h>
#include <OgreVector3.h>
#include <OgreCamera.h>
#include <rviz/frame_position_tracking_view_controller.h>

namespace rviz_carla_plugin
{

CarlaControlPanel::CarlaControlPanel( QWidget* parent )
  : rviz::Panel( parent )
  , linear_velocity_( 0 )
  , angular_velocity_( 0 )
{

  mCarlaStatusSubscriber = nh_.subscribe("/carla/status", 1000, &CarlaControlPanel::carlaStatusChanged, this);
  mCarlaControlPublisher = nh_.advertise<carla_msgs::CarlaControl>("/carla/control", 10);
  mEgoVehicleStatusSubscriber = nh_.subscribe("/carla/ego_vehicle/vehicle_status", 1000, &CarlaControlPanel::egoVehicleStatusChanged, this);
  mEgoVehicleOdometrySubscriber = nh_.subscribe("/carla/ego_vehicle/odometry", 1000, &CarlaControlPanel::egoVehicleOdometryChanged, this);

  mCameraPosePublisher = nh_.advertise<geometry_msgs::PoseStamped>("carla/ego_vehicle/spectator_pose", 10, true);

  mEgoVehicleControlManualOverridePublisher = nh_.advertise<std_msgs::Bool>(
    "/carla/ego_vehicle/vehicle_control_manual_override", 1, true);

  mExecuteScenarioClient = nh_.serviceClient<carla_ros_scenario_runner_types::ExecuteScenario>("/scenario_runner/execute_scenario");
  mScenarioRunnerStatusSubscriber = nh_.subscribe("/scenario_runner/status", 10, &CarlaControlPanel::scenarioRunnerStatusChanged, this);

  mTwistPublisher = nh_.advertise<geometry_msgs::Twist>( "/carla/ego_vehicle/twist", 1 );

  mScenarioSubscriber = nh_.subscribe("/carla/available_scenarios", 1, &CarlaControlPanel::carlaScenariosChanged, this);

  QVBoxLayout* layout = new QVBoxLayout;
  QHBoxLayout* vehicleLayout = new QHBoxLayout;
  
  QFormLayout* egoCtrlStatusLayout = new QFormLayout;

  mThrottleBar = new QProgressBar();
  mThrottleBar->setRange(0, 100);
  egoCtrlStatusLayout->addRow("Throttle", mThrottleBar);
  mBrakeBar = new QProgressBar();
  mBrakeBar->setRange(0, 100);
  egoCtrlStatusLayout->addRow("Brake", mBrakeBar);
  mSteerBar = new QProgressBar();
  mSteerBar->setRange(-100, 100);
  egoCtrlStatusLayout->addRow("Steer", mSteerBar);
  vehicleLayout->addLayout( egoCtrlStatusLayout );

  QFormLayout* egoStatusLayout = new QFormLayout;
  mPosLabel = new QLineEdit();
  mPosLabel->setDisabled(true);
  egoStatusLayout->addRow("Position", mPosLabel);
  
  mSpeedLabel = new QLineEdit();
  mSpeedLabel->setDisabled(true);
  egoStatusLayout->addRow("Speed", mSpeedLabel);
  
  mHeadingLabel = new QLineEdit();
  mHeadingLabel->setDisabled(true);
  egoStatusLayout->addRow("Heading", mHeadingLabel);

  vehicleLayout->addLayout( egoStatusLayout );

  QVBoxLayout* egoCtrlLayout = new QVBoxLayout;
  mOverrideVehicleControl = new QCheckBox("Manual control");
  mOverrideVehicleControl->setDisabled(true);
  egoCtrlLayout->addWidget( mOverrideVehicleControl );
  drive_widget_ = new DriveWidget;
  drive_widget_->setDisabled(true);
  egoCtrlLayout->addWidget( drive_widget_ );
  connect(mOverrideVehicleControl, SIGNAL(stateChanged(int)), this, SLOT(overrideVehicleControl(int)));

  vehicleLayout->addLayout(egoCtrlLayout);

  layout->addLayout(vehicleLayout);


  QFormLayout* carlaLayout = new QFormLayout;

  //row1
  QHBoxLayout* carlaRow1Layout = new QHBoxLayout;

  mScenarioSelection = new QComboBox();
  carlaRow1Layout->addWidget(mScenarioSelection, 10);

  mTriggerScenarioButton = new QPushButton("Execute");
  carlaRow1Layout->addWidget(mTriggerScenarioButton);

  mIndicatorWidget = new IndicatorWidget();
  carlaRow1Layout->addWidget(mIndicatorWidget);
  connect(mTriggerScenarioButton, SIGNAL(released()),this, SLOT(executeScenario()));


  carlaLayout->addRow("Scenario Execution", carlaRow1Layout);


  QHBoxLayout* synchronous_layout = new QHBoxLayout;
  QPixmap pixmap(":/icons/play.png");
  QIcon iconPlay(pixmap);
  mPlayPauseButton = new QPushButton(iconPlay,"");
  synchronous_layout->addWidget( mPlayPauseButton );
  connect(mPlayPauseButton, SIGNAL(released()),this, SLOT(carlaTogglePlayPause()));

  QPixmap pixmapStepOnce(":/icons/step_once.png");
  QIcon iconStepOnce(pixmapStepOnce);
  mStepOnceButton = new QPushButton(iconStepOnce, "");
  connect(mStepOnceButton, SIGNAL(released()),this, SLOT(carlaStepOnce()));

  synchronous_layout->addWidget( mStepOnceButton );
  carlaLayout->addRow("Carla Control", synchronous_layout);

  layout->addLayout( carlaLayout );

  setLayout( layout );

  QTimer* output_timer = new QTimer( this );

  // Next we make signal/slot connections.
  connect( drive_widget_, SIGNAL( outputVelocity( float, float )), this, SLOT( setVel( float, float )));
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));
  output_timer->start( 100 );
  drive_widget_->setEnabled( false );

  //initially set the camera after 1s
  QTimer::singleShot(1000, this, SLOT(updateCameraPos()));
}

void CarlaControlPanel::updateCameraPos()
{
  auto frame = mViewController->subProp("Target Frame")->getValue();
  //TODO: getting the values from the camera might return the old values
  auto camera = mViewController->getCamera();

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = frame.toString().toStdString();
  pose.header.stamp = ros::Time::now();
  pose.pose.position.x = camera->getPosition().x;
  pose.pose.position.y = camera->getPosition().y;
  pose.pose.position.z = camera->getPosition().z;
  pose.pose.orientation.x = camera->getOrientation().x;
  pose.pose.orientation.y = camera->getOrientation().y;
  pose.pose.orientation.z = camera->getOrientation().z;
  pose.pose.orientation.w = camera->getOrientation().w;
  
  mCameraPosePublisher.publish(pose);
}

void CarlaControlPanel::onInitialize()
{
  mViewController = dynamic_cast<rviz::FramePositionTrackingViewController*>(vis_manager_->getViewManager()->getCurrent());
  if (!mViewController) {
    std::cout << "Invalid view controller!" <<std::endl;
    return;
  }

  connect(mViewController->subProp("Target Frame"), SIGNAL(changed()), this, SLOT(updateCameraPos()));
  connect(mViewController->subProp("Focal Point"), SIGNAL(changed()), this, SLOT(updateCameraPos()));
  connect(mViewController->subProp("Distance"), SIGNAL(changed()), this, SLOT(updateCameraPos()));
  connect(mViewController->subProp("Yaw"), SIGNAL(changed()), this, SLOT(updateCameraPos()));
  connect(mViewController->subProp("Pitch"), SIGNAL(changed()), this, SLOT(updateCameraPos()));
  connect(mViewController, SIGNAL(changed()), this, SLOT(updateCameraPos()));
}

void CarlaControlPanel::executeScenario() {
  for (auto const& scenario: mCarlaScenarios->scenarios) {
    if (QString::fromStdString(scenario.name) == mScenarioSelection->currentText()) {
      carla_ros_scenario_runner_types::ExecuteScenario srv;   
      srv.request.scenario = scenario;
      if (!mExecuteScenarioClient.call(srv))
      {
        ROS_ERROR("Failed to call service executeScenario");
      }
      break;
    }
  }
}

void CarlaControlPanel::overrideVehicleControl(int state)
{
  std_msgs::Bool boolMsg;
  if (state == Qt::Checked) {
    boolMsg.data = true;
    drive_widget_->setEnabled(true);
  }
  else {
    boolMsg.data = false;
    drive_widget_->setEnabled(false);
  }
  mEgoVehicleControlManualOverridePublisher.publish(boolMsg);
}

void CarlaControlPanel::scenarioRunnerStatusChanged(const carla_ros_scenario_runner_types::CarlaScenarioRunnerStatus::ConstPtr &msg)
{
  if (msg->status == carla_ros_scenario_runner_types::CarlaScenarioRunnerStatus::STOPPED) {
    mIndicatorWidget->setState(IndicatorWidget::State::Stopped);
  }
  else if (msg->status == carla_ros_scenario_runner_types::CarlaScenarioRunnerStatus::STARTING) {
    mIndicatorWidget->setState(IndicatorWidget::State::Starting);
  }
  else if (msg->status == carla_ros_scenario_runner_types::CarlaScenarioRunnerStatus::RUNNING) {
    mIndicatorWidget->setState(IndicatorWidget::State::Running);
  }
  else if (msg->status == carla_ros_scenario_runner_types::CarlaScenarioRunnerStatus::SHUTTINGDOWN) {
    mIndicatorWidget->setState(IndicatorWidget::State::ShuttingDown);
  }
  else {
    mIndicatorWidget->setState(IndicatorWidget::State::Error);
  }
}

void CarlaControlPanel::carlaScenariosChanged(const carla_ros_scenario_runner_types::CarlaScenarioList::ConstPtr &msg)
{
  auto currentSelection = mScenarioSelection->currentText();
  mCarlaScenarios = msg;
  mScenarioSelection->clear();
  int idx = 0;
  for (auto const& scenario: msg->scenarios) {
    auto name = QString::fromStdString(scenario.name);
    mScenarioSelection->addItem(name);
    if (name == currentSelection) { //switch to previously selected item
      mScenarioSelection->setCurrentIndex(idx);
    }
    ++idx;
  }

}

void CarlaControlPanel::egoVehicleStatusChanged(const carla_msgs::CarlaEgoVehicleStatus::ConstPtr &msg)
{
  mOverrideVehicleControl->setEnabled(true);
  mSteerBar->setValue(msg->control.steer*100);
  mThrottleBar->setValue(msg->control.throttle*100);
  mBrakeBar->setValue(msg->control.brake*100);
  
  std::stringstream speedStream;
  speedStream << std::fixed << std::setprecision(2) << msg->velocity * 3.6;
  mSpeedLabel->setText(speedStream.str().c_str());
}

void CarlaControlPanel::egoVehicleOdometryChanged(const nav_msgs::Odometry::ConstPtr &msg)
{
  std::stringstream posStream;
  posStream << std::fixed << std::setprecision(2) << msg->pose.pose.position.x << ", " << msg->pose.pose.position.y;
  mPosLabel->setText(posStream.str().c_str());

  std::stringstream headingStream;
  headingStream << std::fixed << std::setprecision(2) << tf::getYaw(msg->pose.pose.orientation);
  mHeadingLabel->setText(headingStream.str().c_str());
}

void CarlaControlPanel::carlaStatusChanged(const carla_msgs::CarlaStatus::ConstPtr& msg)
{
  mCarlaStatus = msg;
  if (mCarlaStatus->synchronous_mode) {
      mPlayPauseButton->setDisabled(false);
      mStepOnceButton->setDisabled(false);
      if (mCarlaStatus->synchronous_mode_running) {
          QPixmap pixmap(":/icons/pause.png");
          QIcon iconPause(pixmap);
          mPlayPauseButton->setIcon(iconPause);
      }
      else {
          QPixmap pixmap(":/icons/play.png");
          QIcon iconPlay(pixmap);
          mPlayPauseButton->setIcon(iconPlay);
      }
  }
  else {
      mPlayPauseButton->setDisabled(true);
      mStepOnceButton->setDisabled(true);
  }
}

void CarlaControlPanel::carlaStepOnce() {
  std::cout << "Step once" << std::endl;
  carla_msgs::CarlaControl ctrl;
  ctrl.command = carla_msgs::CarlaControl::STEP_ONCE;
  mCarlaControlPublisher.publish(ctrl);
}

void CarlaControlPanel::carlaTogglePlayPause() {
  if (mCarlaStatus) {
    carla_msgs::CarlaControl ctrl;
    if (mCarlaStatus->synchronous_mode_running) {
        ctrl.command = carla_msgs::CarlaControl::PAUSE;
    }
    else {
        ctrl.command = carla_msgs::CarlaControl::PLAY;
    }
    std::cout << "TOGGLE to " << ctrl.command<<  std::endl;
    mCarlaControlPublisher.publish(ctrl);
  }
}

void CarlaControlPanel::setVel( float lin, float ang )
{
  linear_velocity_ = lin;
  angular_velocity_ = ang;
}

void CarlaControlPanel::sendVel()
{
  if( ros::ok() && mTwistPublisher && (mOverrideVehicleControl->checkState() == Qt::Checked)  )
  {
    geometry_msgs::Twist msg;
    msg.linear.x = linear_velocity_;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = angular_velocity_;
    mTwistPublisher.publish( msg );
  }
}

void CarlaControlPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
 // config.mapSetValue( "Topic", output_topic_ );
}

// Load all configuration data for this panel from the given Config object.
void CarlaControlPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString topic;
  if( config.mapGetString( "Topic", &topic ))
  {
    //output_topic_editor_->setText( topic );
    // updateTopic();
  }
}

} // end namespace rviz_carla_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_carla_plugin::CarlaControlPanel,rviz::Panel )

