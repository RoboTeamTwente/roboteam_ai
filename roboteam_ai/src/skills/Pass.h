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
    int robotToPassToID;
    std::shared_ptr<roboteam_msgs::WorldRobot> robotToPassTo;
    enum progression {
        INITIATING, POSITIONING, KICKING
    };
    progression currentProgress;

    Vector2 targetPos;
    control::ControlGoToPos goToPos;
    GoToType goToType;
public:
    explicit Pass(string name, bt::Blackboard::Ptr blackboard);
    void onInitialize() override;
    Status onUpdate() override;

};

} //ai
} //rtt


#endif //ROBOTEAM_AI_PASS_H
