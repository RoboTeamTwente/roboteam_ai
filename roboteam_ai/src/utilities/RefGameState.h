//
// Created by mrlukasbos on 11-5-19.
//

#ifndef ROBOTEAM_AI_REFGAMESTATE_H
#define ROBOTEAM_AI_REFGAMESTATE_H

#include "Constants.h"

namespace rtt {
namespace ai {

class RefGameState {
private:
    RefCommand commandId;
    RefCommand followUpCommandId;

    std::string strategyName;
    std::string keeperStrategyName;

    // rules for this specific refgamestate
    double maxRobotVel;
    double maxCollisionVel;
    double maxBallVel;
    bool robotsCanEnterDefenseArea;
    bool robotsCanGoOutOfField;
    bool robotsCanTouchBall;
public:
    explicit RefGameState();
    RefCommand getCommandId() const;

    RefGameState(RefCommand commandId, RefCommand followUpCommandId, const std::string &strategyName,
                 const std::string &keeperStrategyName, double maxRobotVel, double maxCollisionVel, double maxBallVel,
                 bool robotsCanEnterDefenseArea, bool robotsCanGoOutOfField, bool robotsCanTouchBall);

    RefCommand getFollowUpCommandId() const;
    const std::string &getStrategyName() const;
    const std::string &getKeeperStrategyName() const;
    double getMaxRobotVel() const;
    double getMaxCollisionVel() const;
    double getMaxBallVel() const;
    bool isRobotsCanEnterDefenseArea() const;
    bool isRobotsCanGoOutOfField() const;
    bool isRobotsCanTouchBall() const;
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_REFGAMESTATE_H
