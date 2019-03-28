//
// Created by mrlukasbos on 24-1-19.
//

#ifndef ROBOTEAM_AI_AVOIDBALLFORBALLPLACEMENT_H
#define ROBOTEAM_AI_AVOIDBALLFORBALLPLACEMENT_H

#include "Skill.h"

namespace rtt {
namespace ai {

class AvoidBall : public Skill {
        const double robotWeight = 0.09;
        const double minRobotDistanceForForce = 0.7;
        const double ballWeight = 0.15;
        const double minBallDistanceForForce = 0.7;
        const double wallWeight = 0.05;
        const double minWallDistanceForForce = 0.4;

    public:
        explicit AvoidBall(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
        bt::Node::Status onUpdate() override;
    private:
        enum Progression {
          RUNNING, DONE, FAIL
        };
        Progression currentProgress;

};

} // ai
} // rtt

#endif //ROBOTEAM_AI_AVOIDBALLFORBALLPLACEMENT_H
