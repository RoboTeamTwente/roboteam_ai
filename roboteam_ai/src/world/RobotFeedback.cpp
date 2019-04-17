//
// Created by mrlukasbos on 17-4-19.
//

#include "RobotFeedback.h"

namespace rtt {
namespace ai {
namespace world {

RobotFeedback::RobotFeedback(const roboteam_msgs::RobotFeedback &copy)
    : robotId(copy.id), batteryState(copy.batteryState), ballSensor(copy.ballSensor), genevaState(copy.genevaDriveState),
    position(copy.position_x, copy.position_y), acceleration(copy.acceleration_x, copy.acceleration_y), feedbackMsg(copy) {
    hasReceivedFeedback = true;
}

} // world
} // ai
} // rtt
