//
// Created by mrlukasbos on 11-5-19.
//

#ifndef ROBOTEAM_AI_RULESET_H
#define ROBOTEAM_AI_RULESET_H

namespace rtt {
namespace ai {

struct RuleSet {
    // rules for this specific refgamestate
    double maxRobotVel;
    double maxCollisionVel;
    double maxBallVel;
    bool robotsCanEnterDefenseArea;
    bool robotsCanGoOutOfField;
    bool robotsCanTouchBall;
};
}
}

#endif //ROBOTEAM_AI_RULESET_H
