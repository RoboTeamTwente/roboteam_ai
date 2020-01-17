//
// Created by rolf on 21/11/18.
//

#ifndef ROBOTEAM_AI_ROTATETOANGLE_H
#define ROBOTEAM_AI_ROTATETOANGLE_H

#include "Skill.h"

namespace rtt::ai {

class RotateToAngle : public Skill {
   private:
    double targetAngle = 0;
    double deltaAngle;
    enum Progression { ROTATING, DONE, FAIL };
    Progression currentProgress;
    Progression checkProgression();

   public:
    explicit RotateToAngle(string name, bt::Blackboard::Ptr blackboard);
    void onInitialize() override;
    Status onUpdate() override;
    void onTerminate(Status s) override;
};
}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_ROTATETOANGLE_H
