//
// Created by robzelluf on 1/22/19.
//

#ifndef ROBOTEAM_AI_RECEIVE_H
#define ROBOTEAM_AI_RECEIVE_H

#include "Skill.h"
#include "InterceptBall.h"

namespace rtt {
namespace ai {

class Receive : public Skill {
private:
    control::ControlGoToPos goToPos;
    GoToType goToType;
    Vector2 focusPoint;
    Vector2 ballStartPos;
    Vector2 ballEndPos;
public:
    explicit Receive(string name, bt::Blackboard::Ptr blackboard);
    void onInitialize() override;
    Status onUpdate() override;

    Vector2 computeInterceptPoint(Vector2 startBall, Vector2 endBall);
};

} //ai
} //rtt

#endif //ROBOTEAM_AI_RECEIVE_H
