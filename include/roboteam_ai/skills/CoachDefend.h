//
// Created by rolf on 5-3-19.
//

#ifndef ROBOTEAM_AI_COACHDEFEND_H
#define ROBOTEAM_AI_COACHDEFEND_H

#include <control/BasicPosControl.h>
#include <control/numtrees/NumTreePosControl.h>
#include "Skill.h"
#include "control/numtrees/NumTreePosControl.h"

namespace rtt::ai {
class CoachDefend : public Skill {
   private:
    bool useBasicGtp(Vector2 targetLocation);

   public:
    explicit CoachDefend(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    void onInitialize() override;
    bt::Node::Status onUpdate() override;
};
}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_COACHDEFEND_H
