//
// Created by mrlukasbos on 23-1-19.
//

#include "EnterFormation.h"
#include "../control/ControlUtils.h"
#include "../utilities/Field.h"

namespace rtt {
namespace ai {

std::vector<std::shared_ptr<roboteam_msgs::WorldRobot>> EnterFormation::robotsInFormation = {};

EnterFormation::EnterFormation(std::string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

void EnterFormation::onInitialize() {
    robotsInFormationMemory = 0;
    // add the robot if its not already there.
    for (auto & current : robotsInFormation) {
        if (current->id == robot->id) {
            return;
        }
    }
    robotsInFormation.push_back(robot);
}

bt::Node::Status EnterFormation::onUpdate() {

    /*
     * Calculate the target location at least once, and every time when the amount of robots in the formation change.
     */
    if (robotsInFormationMemory != robotsInFormation.size()) {
        targetLocation = getFormationPosition();
        robotsInFormationMemory = robotsInFormation.size();
    }
    auto robotPos = rtt::Vector2(robot->pos);
    Vector2 targetToLookAtLocation = Field::get_their_goal_center();

    if (robotPos.dist(targetLocation) > errorMargin) {
        auto velocities = gtp.getPosVelAngle(robot, targetLocation);
        command.x_vel = velocities.vel.x;
        command.y_vel = velocities.vel.y;
        command.w = static_cast<float>((targetLocation-robot->pos).angle());
    } else { // we are at the right location
        command.w = static_cast<float>((targetToLookAtLocation-robot->pos).angle());
    }
    publishRobotCommand();
    return bt::Node::Status::Running;
}

Vector2 EnterFormation::getFormationPosition() {
    auto field = Field::get_field();
    double targetLocationX = field.field_length/4 - (field.field_length/2);

    // first we calculate all the positions for the defense
    std::vector<Vector2> targetLocations;
    std::vector<Vector2> robotLocations;

    for (unsigned int i = 0; i<robotsInFormation.size(); i++) {
        double targetLocationY = ((field.field_width/(robotsInFormation.size() + 1))*(i+1)) - field.field_width/2;
        targetLocations.emplace_back(targetLocationX, targetLocationY);
        robotLocations.emplace_back(robotsInFormation.at(i)->pos);
    }

    // the order of shortestDistances should be the same order as robotLocations
    // this means that shortestDistances[0] corresponds to defenders[0] etc.
    auto shortestDistances = control::ControlUtils::calculateClosestPathsFromTwoSetsOfPoints(robotLocations, targetLocations);

    for (unsigned long i = 0; i<robotsInFormation.size(); i++) {
        if (robotsInFormation.at(i)->id == robot->id) {
            return shortestDistances.at(i).second;
        }
    }
    return {0, 0};
}

void EnterFormation::onTerminate(bt::Node::Status s) {
    for (unsigned long i = 0; i<robotsInFormation.size(); i++) {
        if (robotsInFormation.at(i)->id == robot->id) {
            robotsInFormation.erase(robotsInFormation.begin() + i);
        }
    }
}
} // ai
} // rtt