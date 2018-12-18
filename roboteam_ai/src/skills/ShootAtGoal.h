//
// Created by baris on 13-12-18.
//
#ifndef ROBOTEAM_AI_SHOOTATGOAL_H
#define ROBOTEAM_AI_SHOOTATGOAL_H


#include "Skill.h"
#include <roboteam_ai/src/control/ControlKick.h>

namespace rtt {
namespace ai {

class ShootAtGoal : public Skill {

    public:
        void onInitialize() override;
        Status onUpdate() override;
    private:
        using Command = roboteam_msgs::RobotCommand;
        enum Progression {
          READY, DONE, ROTATING, TURN_GENEVA,
        };

        Progression currentProgress = ROTATING;

        control::ControlKick controlKick;
};
}
}

#endif //ROBOTEAM_AI_SHOOTATGOAL_H
