//
// Created by kjhertenberg on 13-5-19.
//

#ifndef ROBOTEAM_WORLD_KALMANFILTER_H
#define ROBOTEAM_WORLD_KALMANFILTER_H


#include "kalmanObject.h"
#include "roboteam_utils/Position.h"
#include "roboteam_msgs/DetectionFrame.h"
#include "roboteam_msgs/DetectionRobot.h"

namespace rtt {

    void kalmanInit();

    void kalmanUpdate();

    void newFrame(const roboteam_msgs::DetectionFrame msg);

    Position getStates(uint id);

    float getK(uint id);

    void setZ(uint id, float x, float y, float z, double timestamp);

    kalmanObject robotlist[32];

}

#endif //ROBOTEAM_WORLD_KALMANFILTER_H
