//
// Created by robzelluf on 1/22/19.
//

#ifndef ROBOTEAM_AI_RECEIVE_H
#define ROBOTEAM_AI_RECEIVE_H

#include <roboteam_ai/src/control/positionControllers/BasicPosControl.h>
#include "Skill.h"
#include "InterceptBall.h"
#include "roboteam_ai/src/control/PositionUtils.h"

namespace rtt {
namespace ai {

class Receive : public Skill {
private:
    const double RECEIVER_MISSED_BALL_MARGIN = 1.0;
    const double BALL_DEFLECTION_ANGLE = M_PI_4;

    enum Progression {
        POSITIONING,
        RECEIVING
    };

    bool ballPlacement = false;
    bool readyToPassSet = false;

    Progression currentProgress = POSITIONING;

    control::NumTreePosControl numTreeGtp;
    control::BasicPosControl basicGtp;

    Vector2 ballStartPos;
    Vector2 ballEndPos;
    Vector2 ballStartVel;

    BallPtr ballOnPassed;
    bool isBallOnPassedSet = false;

    Vector2 targetPos;

    bool isInPosition(const Vector2& behindTargetPos = {0, 0});
    void moveToCatchPosition(Vector2 position);
    bool passFailed();

public:
    explicit Receive(string name, bt::Blackboard::Ptr blackboard);
    void onInitialize() override;
    Status onUpdate() override;
    void onTerminate(Status s) override;
    Vector2 computeInterceptPoint(const Vector2& startBall, const Vector2& endBall);
    void intercept();
    bool ballDeflected();
};

} //ai
} //rtt

#endif //ROBOTEAM_AI_RECEIVE_H
