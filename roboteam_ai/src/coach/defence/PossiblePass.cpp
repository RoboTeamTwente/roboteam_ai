//
// Created by rolf on 5-4-19.
//

#include <roboteam_ai/src/control/ControlUtils.h>
#include "PossiblePass.h"
#include "roboteam_ai/src/world/Field.h"

namespace rtt{
namespace ai{
namespace coach{
double PossiblePass::distance() {
    return (endPos - startPos).length();
}
bool PossiblePass::obstacleObstructsPath(const Vector2 &obstPos, double obstRadius) {
    return control::ControlUtils::distanceToLineWithEnds(obstPos, startPos, endPos) <= obstRadius;
}
int PossiblePass::amountOfBlockers(const world::WorldData &world) {
    int total = 0;
    for (auto bot : world.them) {
        if (obstacleObstructsPath(bot.pos)) {
            total ++;
        }
    }
    for (auto bot : world.us) {
        if (obstacleObstructsPath(bot.pos)) {
            total ++;
        }
    }
    return total;
}

PossiblePass::PossiblePass(world::Robot *_toBot, const Vector2 &ballPos) :toBot(_toBot),startPos(ballPos){
endPos=botReceivePos(ballPos,_toBot->pos);
}
Vector2 PossiblePass::botReceivePos(const Vector2& _startPos, const Vector2& botPos) {
    Vector2 receivePos =
            botPos + (_startPos - botPos).stretchToLength(Constants::CENTRE_TO_FRONT() + Constants::BALL_RADIUS());
    return receivePos;
}
double PossiblePass::score(const world::WorldData &world) {
    double score = 1.0;
    score *= scoreForGoalAngle(world);
    score *= penaltyForBlocks(world);
    score *= penaltyForDistance();
    return score;
}
double PossiblePass::scoreForGoalAngle(const world::WorldData &world) {
    // find the largest open angle in the world
    std::vector<Line> visibleParts = world::field->getVisiblePartsOfGoal(true, endPos, world);
    std::sort(visibleParts.begin(), visibleParts.end(),
            [](const Line &a, const Line &b) {
              return abs(a.second.y - a.first.y) > abs(b.second.y - b.first.y);
            });
    // set the largest open angle, we use a minimum of 0.05 of the goal angle
    double largestOpenGoalAngle;
    if (visibleParts.empty()) {
        largestOpenGoalAngle = world::field->getTotalGoalAngle(true, endPos)*0.05;
    }
    else {
        double angleOne = (visibleParts[0].first - endPos).angle();
        double angleTwo = (visibleParts[0].second - endPos).angle();
        largestOpenGoalAngle = control::ControlUtils::angleDifference(control::ControlUtils::constrainAngle(angleOne),
                control::ControlUtils::constrainAngle(angleTwo));
    }

    return largestOpenGoalAngle;
}
//Half the score for every robot that blocks the ball
double PossiblePass::penaltyForBlocks(const world::WorldData &world) {
    double obstacleFactor = 0.5;
    return pow(obstacleFactor,amountOfBlockers(world));
}
double PossiblePass::penaltyForDistance() {
    //we use a ramp function has it's zero at goodUntilPassDist and is 1 and impossible Passdist as a penalty
    double goodUntilPassDist = 6.0;
    double impossiblePassDist = 15.0;
    if (distance() > goodUntilPassDist) {
        return (1 - (distance() - goodUntilPassDist)/(impossiblePassDist - goodUntilPassDist));
    }
    return 1.0;

}
}
}
}