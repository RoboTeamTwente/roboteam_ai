//
// Created by baris on 21-2-19.
//

#ifndef ROBOTEAM_AI_GOBEHINDBALL_H
#define ROBOTEAM_AI_GOBEHINDBALL_H

#include "roboteam_ai/src/skills/Skill.h"
#include "roboteam_ai/src/world/Field.h"
#include "GoToPos.h"

namespace rtt {
namespace ai {

class GoBehindBall : public GoToPos {

    private:
        enum RefType {
          penalty,
          freeKick,
          corner
        };

        RefType refType;
        RefType stringToRefType(const std::string &string);
        const double errorMargin = Constants::ROBOT_RADIUS() + 0.05;

    public:
        explicit GoBehindBall(string name, bt::Blackboard::Ptr blackboard);
        Status gtpUpdate() override;
        void gtpInitialize() override;
        void gtpTerminate(Status s) override;

};

} //ai
} //rtt

#endif //ROBOTEAM_AI_GOBEHINDBALL_H
