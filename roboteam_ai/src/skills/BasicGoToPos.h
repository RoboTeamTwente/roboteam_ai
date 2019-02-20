//
// Created by baris on 15-1-19.
//

#ifndef ROBOTEAM_AI_BASICGOTOPOS_H
#define ROBOTEAM_AI_BASICGOTOPOS_H

#include <roboteam_ai/src/control/PositionController.h>
#include "Skill.h"

namespace rtt {
namespace ai {


class BasicGoToPos : public Skill {
    public:
        explicit BasicGoToPos(string name, bt::Blackboard::Ptr blackboard);
        Status onUpdate() override;
        void onInitialize() override;
        Vector2 targetPos;
        control::PositionController goToPos;
        double errorMargin = 0.3;


};
}
}

#endif //ROBOTEAM_AI_BASICGOTOPOS_H
