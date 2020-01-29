//
// Created by mrlukasbos on 1-2-19.
//

#ifndef ROBOTEAM_AI_ROBOTSWIDGET_H
#define ROBOTEAM_AI_ROBOTSWIDGET_H

#include "QLayout"
#include "widget.h"

namespace rtt::ai::interface {

class RobotsWidget : public QWidget {
 Q_OBJECT
 public:
  using Robot = world::Robot;
 private:
  QVBoxLayout *createRobotGroupItem(const Field &field, Robot robot);
  int amountOfSelectedRobots = 0;
  QVBoxLayout *VLayout;
 public:
  explicit RobotsWidget(QWidget *parent);
 public slots:
  void updateContents(Visualizer *visualizer);
};

}  // namespace rtt::ai::interface

#endif  // ROBOTEAM_AI_ROBOTSWIDGET_H
