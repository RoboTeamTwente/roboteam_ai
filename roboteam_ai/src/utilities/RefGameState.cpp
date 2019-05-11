//
// Created by mrlukasbos on 11-5-19.
//

#include "RefGameState.h"

namespace rtt {
namespace ai {

RefGameState::RefGameState() {}

RefGameState::RefGameState(
        RefCommand commandId,
        const std::string &strategyName,
        const std::string &keeperStrategyName,
        RefCommand followUpCommandId,
        double maxRobotVel,
        double maxCollisionVel,
        double maxBallVel,
        bool robotsCanEnterDefenseArea,
        bool robotsCanGoOutOfField,
        bool robotsCanTouchBall)
        : commandId(commandId),
        followUpCommandId(followUpCommandId),
        strategyName(strategyName),
        keeperStrategyName(keeperStrategyName),
        maxRobotVel(maxRobotVel),
        maxCollisionVel(maxCollisionVel),
        maxBallVel(maxBallVel),
        robotsCanEnterDefenseArea(robotsCanEnterDefenseArea),
        robotsCanGoOutOfField(robotsCanGoOutOfField),
        robotsCanTouchBall(robotsCanTouchBall) {}

RefCommand RefGameState::getCommandId() const {
    return commandId;
}

RefCommand RefGameState::getFollowUpCommandId() const {
    return followUpCommandId;
}

const std::string &RefGameState::getStrategyName() const {
    return strategyName;
}

const std::string &RefGameState::getKeeperStrategyName() const {
    return keeperStrategyName;
}

double RefGameState::getMaxRobotVel() const {
    return maxRobotVel;
}

double RefGameState::getMaxCollisionVel() const {
    return maxCollisionVel;
}

double RefGameState::getMaxBallVel() const {
    return maxBallVel;
}

bool RefGameState::isRobotsCanEnterDefenseArea() const {
    return robotsCanEnterDefenseArea;
}

bool RefGameState::isRobotsCanGoOutOfField() const {
    return robotsCanGoOutOfField;
}

bool RefGameState::isRobotsCanTouchBall() const {
    return robotsCanTouchBall;
}

} // ai
} // rtt