//
// Created by robzelluf on 7/2/19.
//

#ifndef ROBOTEAM_AI_FREEKICKPASS_H
#define ROBOTEAM_AI_FREEKICKPASS_H

#include "Pass.h"
#include "Skill.h"

namespace rtt::ai {

class FreeKickPass : public Pass {
   private:
    int maxTries = 3;
    bool forcePass = false;

   public:
    explicit FreeKickPass(std::string name, bt::Blackboard::Ptr blackboard);
    void makeCommand() override;
    void onInitialize() override;
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_FREEKICKPASS_H
