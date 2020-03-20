/*
                    ▄              ▄
                  ▌▒█           ▄▀▒▌
                  ▌▒▒█        ▄▀▒▒▒▐
                 ▐▄▀▒▒▀▀▀▀▄▄▄▀▒▒▒▒▒▐
               ▄▄▀▒░▒▒▒▒▒▒▒▒▒█▒▒▄█▒▐
             ▄▀▒▒▒░░░▒▒▒░░░▒▒▒▀██▀▒▌
            ▐▒▒▒▄▄▒▒▒▒░░░▒▒▒▒▒▒▒▀▄▒▒▌
            ▌░░▌█▀▒▒▒▒▒▄▀█▄▒▒▒▒▒▒▒█▒▐
           ▐░░░▒▒▒▒▒▒▒▒▌██▀▒▒░░░▒▒▒▀▄▌
           ▌░▒▄██▄▒▒▒▒▒▒▒▒▒░░░░░░▒▒▒▒▌
          ▌▒▀▐▄█▄█▌▄░▀▒▒░░░░░░░░░░▒▒▒▐
          ▐▒▒▐▀▐▀▒░▄▄▒▄▒▒▒▒▒▒░▒░▒░▒▒▒▒▌
          ▐▒▒▒▀▀▄▄▒▒▒▄▒▒▒▒▒▒▒▒░▒░▒░▒▒▐
           ▌▒▒▒▒▒▒▀▀▀▒▒▒▒▒▒░▒░▒░▒░▒▒▒▌
           ▐▒▒▒▒▒▒▒▒▒▒▒▒▒▒░▒░▒░▒▒▄▒▒▐
            ▀▄▒▒▒▒▒▒▒▒▒▒▒░▒░▒░▒▄▒▒▒▒▌
              ▀▄▒▒▒▒▒▒▒▒▒▒▄▄▄▀▒▒▒▒▄▀
                ▀▄▄▄▄▄▄▀▀▀▒▒▒▒▒▄▄▀
                   ▒▒▒▒▒▒▒▒▒▒▀▀

    "It's not a robocup if you didn't pull an all-nighter"
                                    - random ER-Force guy
 */

#include <skills/formations/BallPlacementFormation.h>
#include <control/ControlUtils.h>
#include <interface/api/Input.h>
#include <world/FieldComputations.h>
#include "control/ControlUtils.h"
#include <roboteam_utils/Print.h>

namespace rtt::ai {

std::vector<world_new::view::RobotView> BallPlacementFormation::robotsInFormation{};

BallPlacementFormation::BallPlacementFormation(std::string name, bt::Blackboard::Ptr blackboard) : StopFormation(std::move(name), std::move(blackboard)) {
    robotsInFormation = std::vector<world_new::view::RobotView>();
}

Vector2 BallPlacementFormation::getFormationPosition() {
    // failsafe to prevent segfaults
    int amountOfRobots = robotsInFormation.size();
    if (amountOfRobots <= 0) {
        return {};
    }
    auto formationPositions = getStopPositions().at(amountOfRobots - 1);
    std::vector<Vector2> properPositions;
    for (auto pos : formationPositions) {
        if (!positionShouldBeAvoided(pos)) {
            properPositions.push_back(pos);
        }
    }

    int amountToTake = amountOfRobots - properPositions.size();
    std::vector<Vector2> newProposals = getProperPositions(amountToTake);

    for (Vector2 proposal : newProposals) {
        properPositions.push_back(proposal);
    }

    return getOptimalPosition(robot->get()->getId(), robotsInFormation, properPositions);
}

std::vector<world_new::view::RobotView> BallPlacementFormation::robotsInFormationPtr() { return robotsInFormation; }

// adapt to the change of robot amount in formation
void BallPlacementFormation::updateFormation() {
    targetLocation = getFormationPosition();
    robotsInFormationMemory = robotsInFormationPtr().size();
}

bool BallPlacementFormation::positionShouldBeAvoided(Vector2 pos) {
    // get designated position from referee and convert from mm to m
    Vector2 ballPlacementMarker = rtt::ai::GameStateManager::getRefereeDesignatedPosition();

    if (!interface::Output::usesRefereeCommands()) {
        ballPlacementMarker = rtt::ai::interface::Output::getInterfaceMarkerPosition();
        RTT_WARNING("Getting ballplacement location from interface");
    };
    auto ball = world->getBall();
    Vector2 diff = (ball->get()->getPos() - ballPlacementMarker).rotate(M_PI_2);
    interface::Input::drawData(interface::Visual::BALLPLACEMENT, {ball->get()->getPos() + diff.stretchToLength(0.5), ballPlacementMarker + diff.stretchToLength(0.5)}, Qt::magenta,
                               -1, interface::Drawing::LINES_CONNECTED);
    interface::Input::drawData(interface::Visual::BALLPLACEMENT, {ball->get()->getPos() - diff.stretchToLength(0.5), ballPlacementMarker - diff.stretchToLength(0.5)}, Qt::magenta,
                               -1, interface::Drawing::LINES_CONNECTED);
    interface::Input::drawData(interface::Visual::BALLPLACEMENT, {ball->get()->getPos(), ballPlacementMarker}, Qt::magenta, -1, interface::Drawing::REAL_LIFE_CIRCLES, 0.5, 0.5);

    bool tooCloseToLine = control::ControlUtils::distanceToLineWithEnds(pos, Vector2(ball->get()->getPos()), ballPlacementMarker) < 0.9;
    return (tooCloseToLine || !FieldComputations::pointIsInField(*field, pos, 0.0));
}

}  // namespace rtt::ai