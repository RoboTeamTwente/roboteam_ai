//
// Created by baris on 3-4-19.
//

#include <roboteam_ai/src/coach/FormationCoach.h>
#include <roboteam_ai/src/utilities/Field.h>
#include "Stop.h"
#include "../control/ControlUtils.h"
namespace rtt {
namespace ai {

Stop::Stop(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {

}
void Stop::onInitialize() {
    isActive = coach::g_formation.isOffensiveStop(robot->id);
    goToPos.setStop(true);
}
Skill::Status Stop::onUpdate() {

    if (isActive) {
        Vector2 target = getActivePoint();
        command.w = static_cast<float>((target - robot->pos).angle());
        Vector2 velocityRaw = goToPos.getPosVelAngle(robot, target).vel;
        Vector2 velocity = control::ControlUtils::velocityLimiter(velocityRaw, 1.2);
        command.x_vel = static_cast<float>(velocity.x);
        command.y_vel = static_cast<float>(velocity.y);
        publishRobotCommand();
    }
    else {
        // TODO go to fixed positions hungarian
    }

    return Status::Running;


}
void Stop::onTerminate(Skill::Status s) {

    isActive = false;
}
Vector2 Stop::getActivePoint() {

    Vector2 penaltyPos = Field::getPenaltyPoint(true);
    Vector2 ballPos = rtt::ai::World::getBall()->pos;

    Vector2 offset = (penaltyPos - ballPos).stretchToLength(0.6);
    return ballPos + offset;

}
}
}