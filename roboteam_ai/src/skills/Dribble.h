//
// Created by rolf on 28/11/18.
//

#ifndef ROBOTEAM_AI_DRIBBLEBACKWARDS_H
#define ROBOTEAM_AI_DRIBBLEBACKWARDS_H

#include "Skill.h"
#include "../utilities/Constants.h"
#include "../control/ControlUtils.h"

namespace rtt {
namespace ai {

///Dribbles the ball from start position to an end position in a straight line, used for ball placement.
/// Assumes we already have the ball when skill is initialized.
/// Stops at the end to ensure the ball does not spin away.
class Dribble : public Skill {

        const double POS_DIF = 0.05;
        const double SPEED = (Constants::GRSIM() ? 0.4 : 0.8);

    private:
        enum Progression {
          ON_THE_WAY,
          STOPPING,
          STOPPED,
          DONE,
          FAIL,
          WAITING
        };
        Progression currentProgress;
        Progression checkProgression();

        bool forwardDirection;
        int maxTicks = 60;// seconds
        int stopTick = 0;
        int stopTicks = 0;
        int count;
        Vector2 targetPos = {0, 0};
        Vector2 deltaPos;
        double distance;
        float stoppingAngle, initialAngle;

        void sendMoveCommand();
        void sendStopCommand();
    public:
        explicit Dribble(string name, bt::Blackboard::Ptr blackboard);
        void onInitialize() override;
        Status onUpdate() override;
        void onTerminate(Status s) override;
};
}//ai
}//rtt


#endif //ROBOTEAM_AI_DRIBBLEBACKWARDS_H
