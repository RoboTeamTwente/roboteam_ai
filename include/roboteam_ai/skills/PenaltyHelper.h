//
// Created by roboteam on 15-4-19.
//

#ifndef ROBOTEAM_AI_PENALTYHELPER_H
#define ROBOTEAM_AI_PENALTYHELPER_H

#include "Skill.h"

namespace rtt::ai {

class PenaltyHelper : public Skill {
    explicit PenaltyHelper(std::string name, bt::Blackboard::Ptr blackboard);
    Status onUpdate() override;
    void onInitialize() override;
    void onTerminate(Status s) override;
};
}  // namespace rtt::ai
#endif  // ROBOTEAM_AI_PENALTYHELPER_H
