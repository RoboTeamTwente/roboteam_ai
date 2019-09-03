//
// Created by rolf on 5-3-19.
//

#ifndef ROBOTEAM_AI_COACHDEFEND_H
#define ROBOTEAM_AI_COACHDEFEND_H

#include <control/numTrees/NumTreePosControl.h>
#include <control/BasicPosControl.h>
#include "Skill.h"
#include "control/numTrees/NumTreePosControl.h"

namespace rtt {
namespace ai {
class CoachDefend : public Skill {
    private:
        bool useBasicGtp(Vector2 targetLocation);
    public:
        explicit CoachDefend(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
        void onInitialize() override;
        bt::Node::Status onUpdate() override;
};
}
}

#endif //ROBOTEAM_AI_COACHDEFEND_H
