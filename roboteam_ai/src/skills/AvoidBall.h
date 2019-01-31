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

    const double robotWeight = .09;
    const double minRobotDistanceForForce = .7;

    const double ballWeight = .15;
    const double minBallDistanceForForce = .7;

    const double wallWeight = .05;
    const double minWallDistanceForForce = .4;


};

} // ai
} // rtt

#endif //ROBOTEAM_AI_AVOIDBALLFORBALLPLACEMENT_H
