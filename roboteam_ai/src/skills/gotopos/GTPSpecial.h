//
// Created by baris on 15-1-19.
//

#ifndef ROBOTEAM_AI_BASICGOTOPOS_H
#define ROBOTEAM_AI_BASICGOTOPOS_H

#include <roboteam_ai/src/coach/GeneralPositionCoach.h>
#include <roboteam_ai/src/coach/BallplacementCoach.h>
#include <roboteam_ai/src/world/Field.h>
#include "roboteam_ai/src/skills/Skill.h"
#include "GoToPos.h"
#include "roboteam_ai/src/interface/drawer.h"


namespace rtt {
namespace ai {

class GTPSpecial : public GoToPos {
    private:
        enum Type {
            goToBall,
            ballPlacementBefore,
            ballPlacementAfter,
            getBallFromSide,
            defaultType,
        };

        Type type;
        Type stringToType(const std::string& string);

    public:
        explicit GTPSpecial(string name, bt::Blackboard::Ptr blackboard);
        void gtpInitialize() override;
        Status gtpUpdate() override;
        void gtpTerminate(Status s) override;

        double getballFromSideMargin = 0.3;
        Vector2 getBallFromSideLocation();

};
}
}

#endif //ROBOTEAM_AI_BASICGOTOPOS_H
