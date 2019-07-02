//
// Created by robzelluf on 7/2/19.
//

#ifndef ROBOTEAM_AI_FREEKICKPASS_H
#define ROBOTEAM_AI_FREEKICKPASS_H

#include "Skill.h"
#include "Pass.h"

namespace rtt {
namespace ai {

class FreeKickPass : public Pass {
private:
    int maxTries = 3;
    bool forcePass = false;
public:
    explicit FreeKickPass(string name, bt::Blackboard::Ptr blackboard);
    void onInitialize() override;
    Status onUpdate() override;
    void onTerminate(Status s) override;
};

}
}


#endif //ROBOTEAM_AI_FREEKICKPASS_H
