//
// Created by mrlukasbos on 24-1-19.
//

#ifndef ROBOTEAM_AI_AVOIDBALLFORBALLPLACEMENT_H
#define ROBOTEAM_AI_AVOIDBALLFORBALLPLACEMENT_H

#include "Skill.h"

namespace rtt::ai {

class AvoidBall : public Skill {
    const double robotWeight = 0.09;
    double minRobotDistanceForForce;
    const double ballWeight = 0.24;
    const double minBallDistanceForForce = 1.1;
    const double wallWeight = 0.05;
    const double minWallDistanceForForce = 0.4;
    bool stop = false;  // might be useful in the future

   public:
    explicit AvoidBall(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    void onInitialize(std::string type);
    bt::Node::Status onUpdate() override;

   private:
    enum Progression { RUNNING, DONE, FAIL };

    enum Type { BALLPLACEMENT, PASSING, DEFAULT };

    Type type;
    Type stringToType(std::string string);
    std::optional<world_new::view::RobotView> receiver;
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_AVOIDBALLFORBALLPLACEMENT_H
