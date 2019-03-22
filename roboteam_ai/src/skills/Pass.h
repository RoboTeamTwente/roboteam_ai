//
// Created by robzelluf on 1/22/19.
//

#ifndef ROBOTEAM_AI_PASS_H
#define ROBOTEAM_AI_PASS_H

#include "Skill.h"
#include <roboteam_ai/src/coach/PassCoach.h>
#include <roboteam_ai/src/coach/GeneralPositionCoach.h>
#include <roboteam_ai/src/utilities/Constants.h>

namespace rtt {
namespace ai {

class Pass : public Skill {
private:
    coach::PassCoach::PassType type;

    enum PassType {
        onPosition,
        onRobot
    };

    PassType passType;

    int robotToPassToID = -1;
    std::shared_ptr<roboteam_msgs::WorldRobot> robotToPassTo;
    enum Progression {
        POSITIONING, KICKING
    };
    Progression currentProgress;


    Vector2 passPosition;

    Vector2 targetPos;
    control::PositionController goToPos;
    GoToType goToType;

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
