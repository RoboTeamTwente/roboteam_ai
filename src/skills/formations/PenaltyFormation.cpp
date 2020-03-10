//
// Created by baris on 15-4-19.
//

#include <control/PositionUtils.h>
#include <skills/formations/PenaltyFormation.h>

namespace rtt::ai {
std::vector<world_new::view::RobotView> rtt::ai::PenaltyFormation::robotsInFormation{};

rtt::ai::PenaltyFormation::PenaltyFormation(std::string name, bt::Blackboard::Ptr blackboard) : Formation(name, blackboard) {
    robotsInFormation = std::vector<world_new::view::RobotView>();
}

Vector2 rtt::ai::PenaltyFormation::getFormationPosition() {
    std::vector<Vector2> positions;

    if (properties->getBool("Offensive")) {
        positions = rtt::ai::control::PositionUtils::getPenaltyPositions(*field, robotsInFormation.size());
    } else {
        robot->getControllers().getNumTreePosController()->setAvoidBallDistance(0.4);
        positions = rtt::ai::control::PositionUtils::getDefendPenaltyPositions(*field, robotsInFormation.size());
    }

    return getOptimalPosition(robot->get()->getId(), robotsInFormation, positions);
}

std::vector<world_new::view::RobotView> rtt::ai::PenaltyFormation::robotsInFormationPtr() { return robotsInFormation; }

}  // namespace rtt::ai
