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
    int robotToPassToID = -1;
    std::shared_ptr<roboteam_msgs::WorldRobot> robotToPassTo;

    Vector2 targetPos;
    control::PositionController goToPos;

    Status getBall();
    Status moveBehindBall();
    Status shoot();

public:
    explicit Pass(string name, bt::Blackboard::Ptr blackboard);
    void onInitialize() override;
    Status onUpdate() override;

    roboteam_msgs::RobotCommand getBasicCommand() const;
    void sendMoveCommand(const GoToType& goToType, const double minimumSpeed = 0.0);
};

} //ai
} //rtt


#endif //ROBOTEAM_AI_PASS_H
