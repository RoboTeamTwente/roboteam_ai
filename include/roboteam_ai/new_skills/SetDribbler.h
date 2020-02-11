//
// Created by timovdk on 2/11/20.
//

#ifndef RTT_SETDRIBBLER_H
#define RTT_SETDRIBBLER_H

#include <include/roboteam_ai/skills/Skill.h>

namespace rtt::ai {

/**
 * This skill sets the dribbler at a speed specified in the blackboard
 */

class SetDribbler : public Skill {
   public:
    explicit SetDribbler(std::string name, bt::Blackboard::Ptr blackboard);
    void onInitialize() override;
    Status onUpdate() override;
    void onTerminate(Status s) override;
};

}  // namespace rtt::ai

#endif  // RTT_SETDRIBBLER_H
