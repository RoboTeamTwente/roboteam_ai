//
// Created by rolf on 5-4-19.
//

#ifndef ROBOTEAM_AI_POSSIBLEPASS_H
#define ROBOTEAM_AI_POSSIBLEPASS_H

#include <roboteam_utils/Vector2.h>
#include <include/roboteam_ai/world_new/views/RobotView.hpp>
#include "utilities/Constants.h"
#include "world/Field.h"
#include "world/Robot.h"
#include "world/WorldData.h"

namespace rtt::ai::coach {
using namespace rtt::ai::world;

class PossiblePass {
   public:
    world_new::view::RobotView toBot;
    Vector2 startPos;
    Vector2 endPos;
    const double distance();
    bool obstacleObstructsPath(const Vector2 &obstPos, double obstRadius = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS());
    int amountOfBlockers(std::vector<world_new::view::RobotView> us, std::vector<world_new::view::RobotView> them);
    PossiblePass(world_new::view::RobotView _toBot, const Vector2 &ballPos);
    double score(const Field &field, std::vector<world_new::view::RobotView> us, std::vector<world_new::view::RobotView> them);
    // scale from startPos to EndPos
    Vector2 posOnLine(double scale);
    double faceLine();

   private:
    Vector2 botReceivePos(const Vector2 &startPos, const Vector2 &botPos);
    double penaltyForBlocks(std::vector<world_new::view::RobotView> us, std::vector<world_new::view::RobotView> them);
    double penaltyForDistance();
    double scoreForGoalAngle(const Field &field, std::vector<world_new::view::RobotView> us, std::vector<world_new::view::RobotView> them);
};

}  // namespace rtt::ai::coach

#endif  // ROBOTEAM_AI_POSSIBLEPASS_H
