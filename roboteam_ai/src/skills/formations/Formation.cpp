//
// Created by mrlukasbos on 23-1-19.
//

#include <roboteam_ai/src/analysis/GameAnalyzer.h>
#include <roboteam_ai/src/analysis/DecisionMaker.h>
#include "Formation.h"
#include "roboteam_ai/src/control/ControlUtils.h"
#include "roboteam_ai/src/world/Field.h"
#include "roboteam_ai/src/utilities/Hungarian.h"

namespace rtt {
namespace ai {

std::vector<std::shared_ptr<Formation::Robot>> Formation::robotsInFormation = {};

Formation::Formation(std::string name, bt::Blackboard::Ptr blackboard)
: Skill(std::move(name), std::move(blackboard)) {}

void Formation::onInitialize() {
    robotsInFormationMemory = 0;
    addRobotToFormation();
}

bt::Node::Status Formation::onUpdate() {
    if (!robotIsInFormation()) return Status::Running;
    updateFormation(); // if the amount of robots in the formation changes, we want to update the locations


    // if the robot is in position we just need to give it the right angle
    // otherwise it needs to move to the target.
    if (robotIsInPosition()) {
        setFinalAngle();
    } else {
        moveToTarget();
    }

    // send the command
    publishRobotCommand();
    return bt::Node::Status::Running;
}

// determine the angle where the robot should point to (in position)
void Formation::setFinalAngle() {
    Vector2 targetToLookAtLocation = world::field->get_their_goal_center();
    command.w = static_cast<float>((targetToLookAtLocation - robot->pos).angle());
}


void Formation::onTerminate(bt::Node::Status s) {
    removeRobotFromFormation();
}

// loop through all formationrobots to see if our robot is there.
// if not, add it.
void Formation::addRobotToFormation() {
    if (!robotIsInFormation()) robotsInFormation.push_back(robot);
}

// remove robot from formation
void Formation::removeRobotFromFormation() {
    for (int i = 0; i < robotsInFormation.size(); i++) {
        if (robotsInFormation.at(i)->id == robot->id) {
            robotsInFormation.erase(robotsInFormation.begin() + i);
        }
    }
}

// return true if the robot is already in the formation
// NOTE: it does not need to be at the right position yet.
// in that case, use robotIsInPosition()
bool Formation::robotIsInFormation() {
    bool isIn = false;
    for (auto const &bot : robotsInFormation) {
        if (bot->id == robot->id) {
            isIn = true;
        }
    }
    return isIn;
}

// return true if the number of robots in the formation changed.
bool Formation::formationHasChanged() {
    return robotsInFormationMemory != robotsInFormation.size();
}

// adapt to the change of robot amount in formation
void Formation::updateFormation() {
    if (formationHasChanged()) {
        targetLocation = getFormationPosition();
        robotsInFormationMemory = robotsInFormation.size();
    }
}

// return true if a robot is at the desired position
bool Formation::robotIsInPosition() {
    auto robotPos = rtt::Vector2(robot->pos);
    return robotPos.dist(targetLocation) < errorMargin;
}

// set up the command such that a robot moves towards the targetLocation
void Formation::moveToTarget() {
    auto velocities = gtp.getPosVelAngle(robot, targetLocation);
    command.x_vel = velocities.vel.x;
    command.y_vel = velocities.vel.y;
    command.w = static_cast<float>((targetLocation - robot->pos).angle());
}



} // ai
} // rtt