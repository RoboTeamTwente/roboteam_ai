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
        double minRobotDistanceForForce;
        const double ballWeight = 0.15;
        const double minBallDistanceForForce = 0.7;
        const double wallWeight = 0.05;
        const double minWallDistanceForForce = 0.4;
        bool stop = false; // might be useful in the future

    public:
        explicit AvoidBall(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
        void onInitialize() override;
        bt::Node::Status onUpdate() override;
    private:
        enum Progression {
          RUNNING, DONE, FAIL
        };
        Progression currentProgress;

        enum Type {
            BALLPLACEMENT,
            PASSING,
            DEFAULT
        };

        Type type;
        Type stringToType(std::string string);
        RobotPtr receiver;

};

} // ai
} // rtt

#endif //ROBOTEAM_AI_AVOIDBALLFORBALLPLACEMENT_H
