//
// Created by mrlukasbos on 11-5-19.
//

#ifndef ROBOTEAM_AI_RULESET_H
#define ROBOTEAM_AI_RULESET_H

namespace rtt {
namespace ai {

struct RuleSet {

    RuleSet(std::string title, double maxRobotVel, double maxBallVel, double minDistanceToBall, double minDistanceToDefenseArea, bool robotsCanGoOutOfField) :
    title(std::move(title)),
    maxRobotVel(maxRobotVel),
    maxBallVel(maxBallVel),
    minDistanceToBall(minDistanceToBall),
    minDistanceToDefenseArea(minDistanceToDefenseArea),
    robotsCanGoOutOfField(robotsCanGoOutOfField)
    { }

    // rules for this specific refgamestate
    std::string title;
    double maxRobotVel;
    double maxBallVel;
    double minDistanceToBall;
    double minDistanceToDefenseArea;
    bool robotsCanGoOutOfField;

    bool robotsCanEnterDefenseArea() {
        return minDistanceToDefenseArea == -1;
    }

};

}
}

#endif //ROBOTEAM_AI_RULESET_H
