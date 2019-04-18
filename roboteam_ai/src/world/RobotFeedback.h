//
// Created by mrlukasbos on 17-4-19.
//

#ifndef ROBOTEAM_AI_ROBOTFEEDBACK_H
#define ROBOTEAM_AI_ROBOTFEEDBACK_H

#include "roboteam_msgs/RobotFeedback.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {
namespace ai {
namespace world{

class RobotFeedback {
public:
    explicit RobotFeedback();
    explicit RobotFeedback(const roboteam_msgs::RobotFeedback &copy);
private:
    int robotId = -1;
    int batteryState = -1;
    int ballSensor = -1;
    int genevaState = 3;
    rtt::Vector2 position;
    rtt::Vector2 acceleration;
    roboteam_msgs::RobotFeedback feedbackMsg;
    bool hasReceivedFeedback = false;
};

}
}
}

#endif //ROBOTEAM_AI_ROBOTFEEDBACK_H
