#include "Defend.h"
#include "../utilities/Field.h"
#include "../control/ControlUtils.h"

namespace rtt {
namespace ai {

std::vector<std::shared_ptr<roboteam_msgs::WorldRobot>> Defend::allDefenders = {};

Defend::Defend(std::string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

void Defend::onInitialize() {
    allDefenders.push_back(robot);
    targetLocation = getDefensivePosition();
}

bt::Node::Status Defend::onUpdate() {
    auto velocities = gtp.goToPos(robot, targetLocation, control::PosControlType::NUMERIC_TREES);
    roboteam_msgs::RobotCommand cmd;
    cmd.id = robot->id;
    cmd.x_vel = static_cast<float>(velocities.vel.x);
    cmd.y_vel = static_cast<float>(velocities.vel.y);
    cmd.use_angle = 1;
    cmd.w = static_cast<float>((targetLocation - robot->pos).angle());
    publishRobotCommand(cmd);

    return bt::Node::Status::Running;
}

Vector2 Defend::getDefensivePosition() {
    auto field = Field::get_field();
    double targetLocationX = field.field_length/4 - (field.field_length/2);

    // first we calculate all the positions for the defense
    std::vector<Vector2> targetLocations;
    std::vector<Vector2> robotLocations;

    for (unsigned int i = 0; i<allDefenders.size(); i++) {
        double targetLocationY = ((field.field_width/(allDefenders.size() + 1))*(i+1)) - field.field_width/2;
        targetLocations.push_back({targetLocationX, targetLocationY});
        robotLocations.push_back(robot->pos);
    }

    // the order of shortestDistances should be the same order as robotLocations
    // this means that shortestDistances[0] corresponds to defenders[0] etc.
    auto shortestDistances = control::ControlUtils::calculateClosestPathsFromTwoSetsOfPoints(robotLocations, targetLocations);

    for (unsigned long i = 0; i<allDefenders.size(); i++) {
        if (allDefenders.at(i) == robot) {
            return shortestDistances.at(i).second;
        }
    }
    return {0, 0};
}

void Defend::onTerminate(bt::Node::Status s) {
    for (int i = 0; i < allDefenders.size(); i++) {
        if (allDefenders.at(i) == robot) {
            allDefenders.erase(allDefenders.begin() + i);
        }
    }
}

} // ai
} // rtt