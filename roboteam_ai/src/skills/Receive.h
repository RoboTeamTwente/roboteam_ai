//
// Created by robzelluf on 1/22/19.
//

#ifndef ROBOTEAM_AI_RECEIVE_H
#define ROBOTEAM_AI_RECEIVE_H

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

    Progression currentProgress = POSITIONING;

    control::PositionController goToPos;
    GoToType goToType;
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
public:
    explicit Receive(string name, bt::Blackboard::Ptr blackboard);
    void onInitialize() override;
    Status onUpdate() override;
    void onTerminate(Status s) override;
    Vector2 computeInterceptPoint(Vector2 startBall, Vector2 endBall);
};

} //ai
} //rtt

#endif //ROBOTEAM_AI_RECEIVE_H
