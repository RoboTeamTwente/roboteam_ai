//
// Created by thijs on 26-4-19.
//

#ifndef ROBOTEAM_AI_GTPWITHBALL_H
#define ROBOTEAM_AI_GTPWITHBALL_H

#include "include/roboteam_ai/skills/Skill.h"
#include "include/roboteam_ai/control/ballHandling/BallHandlePosControl.h"

namespace rtt {
namespace ai {

/// GTPWithBall should NOT have GoToPos as parent
class GTPWithBall : public Skill {
    private:
        control::BallHandlePosControl ballHandlePosControl;
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
        explicit GTPWithBall(string name, bt::Blackboard::Ptr blackboard);

        void onInitialize() override;
        Status onUpdate() override;
        void onTerminate(Status s) override;
};

}
}

#endif //ROBOTEAM_AI_GTPWITHBALL_H
