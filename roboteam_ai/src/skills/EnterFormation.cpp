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

    // add the robot if its not already there.
    for (unsigned long i = 0; i<robotsInFormation.size(); i++) {
        if (robotsInFormation.at(i)->id == robot->id) {
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
    }


    auto robotPos = rtt::Vector2(robot->pos);
    Vector2 targetToLookAtLocation = Field::get_their_goal_center();

    roboteam_msgs::RobotCommand cmd;
    cmd.id = robot->id;
    cmd.use_angle = 1;

    if (robotPos.dist(targetLocation) > Constants::NUMTREE_ERROR_MARGIN()) {
        auto velocities = gtp.goToPos(robot, targetLocation, control::PosControlType::NUMERIC_TREES);
        cmd.x_vel = velocities.vel.x;
        cmd.y_vel = velocities.vel.y;
        cmd.w = static_cast<float>((targetLocation-robot->pos).angle());
    } else { // we are at the right location
        cmd.w = static_cast<float>((targetToLookAtLocation-robot->pos).angle());
    }
    publishRobotCommand(cmd);
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
        targetLocations.push_back({targetLocationX, targetLocationY});
        robotLocations.push_back(robotsInFormation.at(i)->pos);
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
    //https://en.wikipedia.org/wiki/Erase%E2%80%93remove_idiom
    robotsInFormation.erase(std::remove(robotsInFormation.begin(), robotsInFormation.begin(), robot), robotsInFormation.end());
}
} // ai
} // rtt