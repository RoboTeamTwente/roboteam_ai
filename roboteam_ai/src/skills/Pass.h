//
// Created by robzelluf on 1/22/19.
//

#ifndef ROBOTEAM_AI_PASS_H
#define ROBOTEAM_AI_PASS_H

#include <roboteam_ai/src/control/positionControllers/PosController.h>
#include "Skill.h"

namespace rtt {
namespace ai {

class Pass : public Skill {
private:
    int robotToPassToID = -1;
    std::shared_ptr<roboteam_msgs::WorldRobot> robotToPassTo;
    enum Progression {
        POSITIONING, KICKING
    };
    Progression currentProgress;

    Vector2 targetPos;

    control::NumTreePosControl numTreeGtp;
    control::BasicPosControl basicGtp;

    double distance;
    double kicker_vel_multiplier;

    int checkTicks;
    int maxCheckTicks = 20;
public:
    explicit Pass(string name, bt::Blackboard::Ptr blackboard);
    void onInitialize() override;
    Status onUpdate() override;

};

} //ai
} //rtt


#endif //ROBOTEAM_AI_PASS_H
