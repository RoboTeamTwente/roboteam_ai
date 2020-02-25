//
// Created by robzelluf on 5/13/19.
//

#ifndef ROBOTEAM_AI_DRIBBLEFORWARD_H
#define ROBOTEAM_AI_DRIBBLEFORWARD_H

#include "Skill.h"

namespace rtt::ai {

class DribbleForward : public Skill {
   private:
    double dribbleDistance = 0.8;
    Vector2 targetPos;

   public:
    explicit DribbleForward(std::string name, bt::Blackboard::Ptr blackboard);
    void onInitialize() override;
    Status onUpdate() override;
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_DRIBBLEFORWARD_H
