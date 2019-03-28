//
// Created by baris on 14-3-19.
//

#ifndef ROBOTEAM_AI_SHOOTFREEKICK_H
#define ROBOTEAM_AI_SHOOTFREEKICK_H

#include <roboteam_ai/src/control/positionControllers/BasicPosControl.h>
#include "Skill.h"
#include "../utilities/Field.h"


namespace rtt {
namespace ai {

class ShootFreeKick : public Skill {

    public:
        explicit ShootFreeKick(string name, bt::Blackboard::Ptr blackboard);
        Status onUpdate() override;
        void onInitialize() override;
        void onTerminate(Status s) override;


    private:
        enum Progress {
          GOING,
          TARGETING,
          READY,
          SHOOTING
        };

        Progress progress;
        Vector2 targetPos;
        control::BasicPosControl goToPos;
        double errorMarginPos = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS() + 0.03; // Same logic
        bool isShot();
        Vector2 freeKickPos;

};

}
}

#endif //ROBOTEAM_AI_SHOOTFREEKICK_H
