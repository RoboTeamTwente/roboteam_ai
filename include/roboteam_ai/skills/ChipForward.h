//
// Created by robzelluf on 7/5/19.
//

#ifndef ROBOTEAM_AI_CHIPFORWARD_H
#define ROBOTEAM_AI_CHIPFORWARD_H

#include "Skill.h"

namespace rtt::ai {

class ChipForward : public Skill {
   private:
    Vector2 aimPoint;
    bool hasChipped = false;

   public:
    explicit ChipForward(std::string name, bt::Blackboard::Ptr blackboard);
    Status onUpdate() override;
    void onInitialize() override;
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_CHIPFORWARD_H
