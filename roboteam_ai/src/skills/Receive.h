//
// Created by robzelluf on 1/22/19.
//

#ifndef ROBOTEAM_AI_RECEIVE_H
#define ROBOTEAM_AI_RECEIVE_H

#include <roboteam_ai/src/control/positionControllers/BasicPosControl.h>
#include "Skill.h"
#include "InterceptBall.h"
#include "../coach/GeneralPositionCoach.h"

namespace rtt {
namespace ai {

class Receive : public Skill {
private:
    enum Progression {
        POSITIONING,
        RECEIVING
    };

    bool ballPlacement = false;
    bool hasTargetLocation = false;
    Vector2 targetLocation;

    Progression currentProgress = POSITIONING;

    control::NumTreePosControl numTreeGtp;
    control::BasicPosControl basicGtp;

    Vector2 focusPoint;
    Vector2 ballStartPos;
    Vector2 ballEndPos;
    Vector2 ballStartVel;
    bool initializedBall;

    int checkTicks;
    int maxCheckTicks = 20;

    int stopDribbleTick = 0;
    int stopDribbleTicks = 20;

    Vector2 targetPos;
    Vector2 passPosition;
    bool readyToReceivePass = false;


    bool isInPosition(const Vector2& behindTargetPos = {0, 0});
    void moveToCatchPosition(Vector2 position);

public:
    explicit Receive(string name, bt::Blackboard::Ptr blackboard);
    void onInitialize() override;
    Status onUpdate() override;
    void onTerminate(Status s) override;
    Vector2 computeInterceptPoint(const Vector2& startBall, const Vector2& endBall);
    void intercept();
};

} //ai
} //rtt

#endif //ROBOTEAM_AI_RECEIVE_H
