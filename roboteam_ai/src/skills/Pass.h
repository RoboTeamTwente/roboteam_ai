//
// Created by robzelluf on 1/22/19.
//

#ifndef ROBOTEAM_AI_PASS_H
#define ROBOTEAM_AI_PASS_H

#include "Skill.h"
#include "../utilities/Coach.h"

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
    control::ControlGoToPos goToPos;
    GoToType goToType;

    double maxDistance = 9.0;
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
