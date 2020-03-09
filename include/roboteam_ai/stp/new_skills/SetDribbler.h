//
// Created by timovdk on 2/11/20.
//

#ifndef RTT_SETDRIBBLER_H
#define RTT_SETDRIBBLER_H

#include "include/roboteam_ai/stp/Skill.h"

namespace rtt::ai::stp {

/**
 * This skill sets the dribbler at a speed specified in the blackboard
 */
class SetDribbler : public Skill {
   public:
        Status onInitialize() noexcept override;
        /**
        * Sets the dribbler speed using blackboard parameter: "dribblerSpeed"
        * @return status of the skill
        */
        Status onUpdate(SkillInfo const& info) noexcept override;
        Status onTerminate() noexcept override;
};

}  // namespace rtt::ai

#endif  // RTT_SETDRIBBLER_H
