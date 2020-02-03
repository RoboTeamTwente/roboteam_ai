//
// Created by rolf on 28/11/18.
//

#ifndef ROBOTEAM_AI_DRIBBLEBACKWARDS_H
#define ROBOTEAM_AI_DRIBBLEBACKWARDS_H

#include "Skill.h"
#include "control/ControlUtils.h"
#include "control/ball-handling/BallHandlePosControl.h"
#include "utilities/Constants.h"

namespace rtt::ai {

/// Dribbles the ball from start position to an end position in a straight line, used for ball placement.
/// Assumes we already have the ball when skill is initialized.
/// Stops at the end to ensure the ball does not spin away.
    class Dribble : public Skill {
        private:
        control::BallHandlePosControl::TravelStrategy forwardDirection;
        int maxTicks = 60;  // seconds
        int count;
        Vector2 targetPos = {0, 0};
        double distance;

        public:
        explicit Dribble(std::string name, bt::Blackboard::Ptr blackboard);
        void onInitialize() override;
        Status onUpdate() override;
    };
}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_DRIBBLEBACKWARDS_H
