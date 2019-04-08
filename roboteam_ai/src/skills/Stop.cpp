//
// Created by baris on 3-4-19.
//

#include <roboteam_ai/src/coach/FormationCoach.h>
#include <roboteam_ai/src/world/Field.h>
#include "Stop.h"
#include "../control/ControlUtils.h"
namespace rtt {
namespace ai {

using Robot = rtt::ai::world::Robot;
std::vector<int> Stop::robotsInFormation = {};
int Stop::defensiveOffensive = -1;
int Stop::numberInFormation = 0;
map<int, Vector2> Stop::shortestDistances ={};

Stop::Stop(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {

}
void Stop::onInitialize() {
    isActive = coach::g_formation.isOffensiveStop(robot->id);
    goToPos.setStop(true);
    if (isActive)
        return;

    robotsInFormation.push_back(robot->id);
}
Skill::Status Stop::onUpdate() {

    if (isActive) {
        if (defensiveOffensive == -1 || defensiveOffensive == robot->id) {
            defensiveOffensive = robot->id;
            targetLocation = getDefensiveActivePoint();
        }
        else{
            targetLocation = getOffensiveActivePoint();
        }
        command.w = static_cast<float>((targetLocation - robot->pos).angle());
        Vector2 velocityRaw = goToPos.getPosVelAngle(robot, targetLocation).vel;
        Vector2 velocity = control::ControlUtils::velocityLimiter(velocityRaw, 1.2);
        command.x_vel = static_cast<float>(velocity.x);
        command.y_vel = static_cast<float>(velocity.y);
        publishRobotCommand();
    }
    else {

        auto robotPos = rtt::Vector2(robot->pos);
        Vector2 targetToLookAtLocation = rtt::ai::world::field->get_their_goal_center();
        targetLocation = getFormationPosition();

        if (robotPos.dist(targetLocation) > 0.08) {
            auto velocities = goToPos.getPosVelAngle(robot, targetLocation);
            command.x_vel = velocities.vel.x;
            command.y_vel = velocities.vel.y;
            command.w = static_cast<float>((targetLocation-robot->pos).angle());
        } else { // we are at the right location
            command.w = static_cast<float>((targetToLookAtLocation-robot->pos).angle());
        }
        publishRobotCommand();
    }

    return Status::Running;

}
void Stop::onTerminate(Skill::Status s) {

    isActive = false;
}
Vector2 Stop::getOffensiveActivePoint() {

    Vector2 penaltyPos = rtt::ai::world::field->getPenaltyPoint(false);
    Vector2 ballPos = rtt::ai::world::world->getBall()->pos;

    Vector2 offset = (penaltyPos - ballPos).stretchToLength(0.6);
    return ballPos + offset;

}

Vector2 Stop::getDefensiveActivePoint() {

    Vector2 penaltyPos = rtt::ai::world::field->getPenaltyPoint(false);
    Vector2 ballPos = rtt::ai::world::world->getBall()->pos;

    Vector2 offset = (penaltyPos - ballPos).stretchToLength(0.6);
    return ballPos + offset;

}

Vector2 Stop::getFormationPosition() {

if (numberInFormation != robotsInFormation.size()) {
    rtt::HungarianAlgorithm hungarian;
    shortestDistances = hungarian.getRobotPositions(robotsInFormation, true, coach::g_formation.getStopPositions());
    numberInFormation = robotsInFormation.size();
}
    return shortestDistances[robot->id];


}
}
}