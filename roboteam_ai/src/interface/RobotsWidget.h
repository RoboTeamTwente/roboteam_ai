//
// Created by mrlukasbos on 1-2-19.
//

#ifndef ROBOTEAM_AI_ROBOTSWIDGET_H
#define ROBOTEAM_AI_ROBOTSWIDGET_H

#include "QLayout"
#include "widget.h"

namespace rtt {
namespace ai {
namespace interface {

class RobotsWidget: public QHBoxLayout {
Q_OBJECT
private:
    void clearLayout(QLayout* layout);
    QVBoxLayout* createRobotGroupItem(roboteam_msgs::WorldRobot robot);
    int amountOfSelectedRobots = 0;
public:
    void updateContents(Visualizer* visualizer);
};

}
}
}

#endif //ROBOTEAM_AI_ROBOTSWIDGET_H
