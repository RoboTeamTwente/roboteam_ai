//
// Created by baris on 3-4-19.
//

#include <roboteam_ai/src/coach/FormationCoach.h>
#include <roboteam_ai/src/utilities/Field.h>
#include "Stop.h"
#include "../control/ControlUtils.h"
namespace rtt {
namespace ai {
std::vector<std::shared_ptr<roboteam_msgs::WorldRobot>> Stop::robotsInFormation = {};
int Stop::defensiveOffensive = -1;

Stop::Stop(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {

}
void Stop::onInitialize() {
    isActive = coach::g_formation.isOffensiveStop(robot->id);
    goToPos.setStop(true);
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

        if (robotsInFormationMemory != robotsInFormation.size()) {
            targetLocation = getFormationPosition();
            robotsInFormationMemory = robotsInFormation.size();
        }
        auto robotPos = rtt::Vector2(robot->pos);
        Vector2 targetToLookAtLocation = Field::get_their_goal_center();

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

    Vector2 penaltyPos = Field::getPenaltyPoint(false);
    Vector2 ballPos = rtt::ai::World::getBall()->pos;

    Vector2 offset = (penaltyPos - ballPos).stretchToLength(0.6);
    return ballPos + offset;

}

Vector2 Stop::getDefensiveActivePoint() {

    Vector2 penaltyPos = Field::getPenaltyPoint(true);
    Vector2 ballPos = rtt::ai::World::getBall()->pos;

    Vector2 offset = (penaltyPos - ballPos).stretchToLength(0.6);
    return ballPos + offset;

}

Vector2 Stop::getFormationPosition() {
    // first we calculate all the positions for the defense
    std::vector<Vector2> targetLocations = coach::g_formation.getStopPositions();
    std::vector<Vector2> robotLocations;

    for (auto & i : robotsInFormation) {
        std::cout << "t" << std::endl;
        robotLocations.emplace_back(i->pos);
    }

    // Hungarian
    auto shortestDistances = control::ControlUtils::calculateClosestPathsFromTwoSetsOfPoints(robotLocations, targetLocations);

    // Get the point through the hungarian
    for (unsigned long i = 0; i<robotsInFormation.size(); i++) {
        if (robotsInFormation.at(i)->id == robot->id) {
            return shortestDistances.at(i).second;
        }
    }
    return {0, 0};
}
}
}