//
// Created by baris on 8-4-19.
//

#include <roboteam_ai/src/coach/FormationCoach.h>
#include "PassiveStop.h"
namespace rtt{
namespace ai {

bool PassiveStop::done = false;
map<int, Vector2> PassiveStop::shortestDistances;
PassiveStop::PassiveStop(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {

}
void PassiveStop::onInitialize() {
    goToPos.setStop(true);
    coach::g_formation.registerPassive(robot->id);
}


Skill::Status PassiveStop::onUpdate() {
    if (!done)
        coach::g_formation.makePassivePositions();
    checkBall();

    command.w = static_cast<float>((targetPos - robot->pos).angle());
    Vector2 velocityRaw = goToPos.getPosVelAngle(robot, targetPos).vel;
    Vector2 velocity = control::ControlUtils::velocityLimiter(velocityRaw, 1.2);
    command.x_vel = static_cast<float>(velocity.x);
    command.y_vel = static_cast<float>(velocity.y);
    publishRobotCommand();


    return Status::Running;
}
void PassiveStop::onTerminate(Skill::Status s) {
    done = false;
    coach::g_formation.terminate();
}

Vector2 PassiveStop::getTargetPos() {
    if (done)
        return shortestDistances[robot->id];

    rtt::HungarianAlgorithm hungarian;
    shortestDistances = hungarian.getRobotPositions(coach::g_formation.getPassiveRobots(), true, coach::g_formation.getStopPositions());
    done = true;
    return shortestDistances[robot->id];

}
void PassiveStop::checkBall() {
    // might osilate no promises
    Vector2 ballPos = (rtt::ai::world::world->getBall()->pos);

    if (( ballPos- (robot->pos)).length() < 0.666) {
        if (ballPos.y >= 0.0) {
            Vector2 vec = getTargetPos() - rtt::ai::world::field->get_field().left_penalty_line.end;
            vec = vec.rotate(M_PI_4);
            targetPos = vec + rtt::ai::world::field->get_field().left_penalty_line.end;
        }
        else{

            Vector2 vec = getTargetPos() - rtt::ai::world::field->get_field().left_penalty_line.begin;
            vec = vec.rotate(M_PI_4);
            targetPos = vec + rtt::ai::world::field->get_field().left_penalty_line.end;
        }
    }
    else{
        targetPos = getTargetPos();

    }



}
}
}