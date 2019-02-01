//
// Created by mrlukasbos on 24-1-19.
//

#ifndef ROBOTEAM_AI_AVOIDBALLFORBALLPLACEMENT_H
#define ROBOTEAM_AI_AVOIDBALLFORBALLPLACEMENT_H

#include "Skill.h"

namespace rtt {
namespace ai {

class AvoidBall : public Skill {

public:
    explicit AvoidBall(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    bt::Node::Status onUpdate() override;
private:
    control::ControlGoToPos gtp;
    rtt::Vector2 ballPlacementTargetLocation;
    rtt::Vector2 targetToMoveTo;
    enum Progression {
        RUNNING, DONE, FAIL
    };
    Progression currentProgress;
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_AVOIDBALLFORBALLPLACEMENT_H
