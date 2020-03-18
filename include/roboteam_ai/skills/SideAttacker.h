//
// Created by thijs on 9-1-19.
//

#ifndef ROBOTEAM_AI_SIDEATTACKER_H
#define ROBOTEAM_AI_SIDEATTACKER_H

#include "Skill.h"

namespace rtt::ai {

class SideAttacker : public Skill {
   private:
    Vector2 targetPos;

    Vector2 getOffensivePosition(const rtt::ai::world::Field &field);

   public:
    explicit SideAttacker(std::string name, bt::Blackboard::Ptr blackboard);
    void onInitialize() override;
    Status onUpdate() override;
    void onTerminate(Status s) override;
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_SIDEATTACKER_H
