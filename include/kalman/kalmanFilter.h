//
// Created by kjhertenberg on 13-5-19.
//

#ifndef ROBOTEAM_WORLD_KALMANFILTER_H
#define ROBOTEAM_WORLD_KALMANFILTER_H


#include "kalman/kalmanObject.h"
#include "roboteam_msgs/DetectionFrame.h"
#include "roboteam_msgs/DetectionRobot.h"

void kalmanInit();

void kalmanUpdate();

void newFrame();

kalmanObject robotlist[32];

#endif //ROBOTEAM_WORLD_KALMANFILTER_H
