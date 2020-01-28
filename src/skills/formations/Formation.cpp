#include "skills/formations/Formation.h"

#include <analysis/DecisionMaker.h>
#include <analysis/GameAnalyzer.h>

#include "control/ControlUtils.h"
#include "roboteam_utils/Hungarian.h"
#include "world/Field.h"

namespace rtt::ai {

bool Formation::update = false;
int Formation::updateCount = 0;

Formation::Formation(std::string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

void Formation::onInitialize() {
    robotsInFormationMemory = 0;
    addRobotToFormation();
}

bt::Node::Status Formation::onUpdate() {
    if (!robotIsInFormation()) return Status::Failure;
    updateFormation();  // if the amount of robots in the formation changes, we want to update the locations

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
    Vector2 targetToLookAtLocation = field->get_field().get(THEIR_GOAL_CENTER);
    command.set_w(static_cast<float>((targetToLookAtLocation - robot->pos).angle()));
}

void Formation::terminate(Status s) { onTerminate(s); }

void Formation::onTerminate(bt::Node::Status s) {
    if (robot) {
        removeRobotFromFormation();
    }
}

// loop through all formationrobots to see if our robot is there.
// if not, add it.
void Formation::addRobotToFormation() {
    if (!robotIsInFormation()) robotsInFormationPtr()->push_back(robot);
}

// remove robot from formation
void Formation::removeRobotFromFormation() {
    for (unsigned int i = 0; i < robotsInFormationPtr()->size(); i++) {
        if (robotsInFormationPtr()->at(i) && robotsInFormationPtr()->at(i)->id == robot->id) {
            robotsInFormationPtr()->erase(robotsInFormationPtr()->begin() + i);
        }
    }
}

// return true if the robot is already in the formation
// NOTE: it does not need to be at the right position yet.
// in that case, use robotIsInPosition()
bool Formation::robotIsInFormation() {
    bool isIn = false;
    for (auto const &bot : *robotsInFormationPtr()) {
        if (bot->id == robot->id) {
            isIn = true;
        }
    }
    return isIn;
}

// return true if the number of robots in the formation changed.
bool Formation::formationHasChanged() { return robotsInFormationMemory != static_cast<int>(robotsInFormationPtr()->size()); }

// adapt to the change of robot amount in formation
void Formation::updateFormation() {
    if (formationHasChanged() || updateCounter()) {
        targetLocation = getFormationPosition();
        robotsInFormationMemory = robotsInFormationPtr()->size();
    }
}

// return true if a robot is at the desired position
bool Formation::robotIsInPosition() {
    auto robotPos = rtt::Vector2(robot->pos);
    return robotPos.dist(targetLocation) < errorMargin;
}

// set up the command such that a robot moves towards the targetLocation
void Formation::moveToTarget() {
    auto velocities = robot->getNumtreePosControl()->getRobotCommand(world, field, robot, targetLocation);
    command.mutable_vel()->set_x(velocities.vel.x);
    command.mutable_vel()->set_y(velocities.vel.y);
    command.set_w(static_cast<float>((targetLocation - robot->pos).angle()));
}

bool Formation::updateCounter() {
    if (!update) return false;
    return (++updateCount % 200) == 0;
}

Vector2 Formation::getOptimalPosition(int robotId, const vector<RobotPtr>& robots, std::vector<Vector2> targetLocations) {
  std::unordered_map<int, Vector2> robotLocations;

  for (auto formationRobot : robots) {
    robotLocations.insert({formationRobot->id, formationRobot->pos});
  }

  auto shortestDistances = rtt::Hungarian::getOptimalPairsIdentified(robotLocations, std::move(targetLocations));
  return shortestDistances.at(robotId);
}

}  // namespace rtt::ai