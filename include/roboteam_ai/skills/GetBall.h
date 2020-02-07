//
// Created by rolf on 04/12/18.
//

#ifndef ROBOTEAM_AI_GETBALL_H
#define ROBOTEAM_AI_GETBALL_H

#include "Skill.h"
#include "control/ball-handling/BallHandlePosControl.h"

namespace rtt::ai {

class GetBall : public Skill {
   private:
    control::BallHandlePosControl ballHandlePosControl;
    Vector2 lockedTargetPos = Vector2();

   public:
    explicit GetBall(string name, bt::Blackboard::Ptr blackboard);
    void onInitialize() override;
    Status onUpdate() override;
    void onTerminate(Status s) override;
};
}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_GETBALL_H
