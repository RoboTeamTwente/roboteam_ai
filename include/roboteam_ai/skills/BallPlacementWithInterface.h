//
// Created by thijs on 15-5-19.
//

#ifndef ROBOTEAM_AI_BALLPLACEMENTWITHINTERFACE_H
#define ROBOTEAM_AI_BALLPLACEMENTWITHINTERFACE_H

#include <control/ball-handling/BallHandlePosControl.h>
#include "Skill.h"

namespace rtt::ai {

    class BallPlacementWithInterface : public Skill {
        public:
        explicit BallPlacementWithInterface(string name, bt::Blackboard::Ptr blackboard);
        Status onUpdate() override;

        private:
        control::BallHandlePosControl ballHandlePosControl;
        Vector2 previousTargetPos = Vector2();
    };

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_BALLPLACEMENTWITHINTERFACE_H
