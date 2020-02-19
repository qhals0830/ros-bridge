#pragma once

#include <QWidget>

namespace rviz_carla_plugin
{

class IndicatorWidget: public QWidget
{
Q_OBJECT
public:
  enum class State {
    Stopped,
    Starting,
    Running,
    ShuttingDown,
    Error
  };

  IndicatorWidget( QWidget* parent = 0 );

  virtual void paintEvent( QPaintEvent* event ) override;

  void setState(IndicatorWidget::State state);

private:
  State mCurrentState{State::Stopped};
};

} // end namespace rviz_carla_plugin
