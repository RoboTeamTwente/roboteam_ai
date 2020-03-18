//
// Created by thijs on 26-4-19.
//

#ifndef ROBOTEAM_AI_GTPWITHBALL_H
#define ROBOTEAM_AI_GTPWITHBALL_H

#include "skills/Skill.h"

namespace rtt::ai {

/// GTPWithBall should NOT have GoToPos as parent
class GTPWithBall : public Skill {
   private:
    Vector2 targetPos;
    Angle targetAngle;

    enum TargetType {
        rotateToTheirGoal,
        ballPlacement,

    };
    TargetType targetType;
    TargetType stringToTargetType(const std::string &string);

    void updateTarget();

   public:
    explicit GTPWithBall(std::string name, bt::Blackboard::Ptr blackboard);

    void onInitialize() override;
    Status onUpdate() override;
    void onTerminate(Status s) override;
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_GTPWITHBALL_H
