#include "Defend.h"
#include "../world/Field.h"
#include "../control/ControlUtils.h"

namespace rtt {
namespace ai {

std::vector<std::shared_ptr<Defend::Robot>> Defend::allDefenders = {};

Defend::Defend(std::string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

void Defend::onInitialize() {
    allDefendersMemory = 0;
    // add the robot if its not already there.
    for (unsigned long i = 0; i<allDefenders.size(); i++) {
        if (allDefenders.at(i)->id == robot->id) {
            return;
        }
    }
    allDefenders.push_back(robot);
    gtp.setAvoidBall(0.1);
}

bt::Node::Status Defend::onUpdate() {

    /*
 * Calculate the target location at least once, and every time when the amount of robots in the formation change.
 */
    if (allDefendersMemory != allDefenders.size()) {
        targetLocation = getDefensivePosition();
        allDefendersMemory = allDefenders.size();
    }

    auto velocities = gtp.getPosVelAngle(robot, targetLocation);
    command.x_vel = static_cast<float>(velocities.vel.x);
    command.y_vel = static_cast<float>(velocities.vel.y);
    command.w = static_cast<float>((targetLocation - robot->pos).angle());
    publishRobotCommand();

    return bt::Node::Status::Running;
}

Vector2 Defend::getDefensivePosition() {
    auto field = world::field->get_field();
    double targetLocationX = field.field_length/4 - (field.field_length/2);

    // first we calculate all the positions for the defense
    std::vector<Vector2> targetLocations;
    std::vector<int> robotIds;

    for (unsigned int i = 0; i<allDefenders.size(); i++) {
        double targetLocationY = ((field.field_width/(allDefenders.size() + 1))*(i+1)) - field.field_width/2;
        targetLocations.push_back({targetLocationX, targetLocationY});
        robotIds.push_back(allDefenders.at(i)->id);
    }

    rtt::HungarianAlgorithm hungarian;
    auto shortestDistances = hungarian.getRobotPositions(robotIds, true, targetLocations);

    return shortestDistances.at(robot->id);
}

void Defend::onTerminate(bt::Node::Status s) {
    for (int i = 0; i < allDefenders.size(); i++) {
        if (allDefenders.at(i)->id == robot->id) {
            allDefenders.erase(allDefenders.begin() + i);
        }
    }
}

} // ai
} // rtt