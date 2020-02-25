//
// Created by rolf on 18-2-19.
//

#ifndef ROBOTEAM_AI_DEFENSIVECOACH_H
#define ROBOTEAM_AI_DEFENSIVECOACH_H

#include <gtest/gtest_prod.h>
#include <roboteam_utils/Line.h>
#include <roboteam_utils/Vector2.h>
#include "PossiblePass.h"
#include "world/Field.h"
#include "world_new/views/WorldDataView.hpp"

namespace rtt::ai::coach {
enum botType { BLOCKBALL, BLOCKTOGOAL, BLOCKPASS, BLOCKONLINE };

struct DefenderBot {
    int id;
    Vector2 targetPos;
    double orientation;
    int blockFromID;
    botType type;

    int coveredCount = 0;
    const world_new::view::RobotView toRobot();
};
class DefencePositionCoach {
    FRIEND_TEST(defensive_coach, blockPoints);

 public:
    DefencePositionCoach() = default;
    double maxX(const Field &field);  // furthest point forwards the availableIDs can go

    Vector2 getMostDangerousPos();

    DefenderBot createBlockBall(const Field &field, const Line &blockLine);
    DefenderBot createBlockToGoal(const Field &field, const PossiblePass &pass, double aggressionFactor, const Line &blockLine);
    DefenderBot createBlockToGoal(const Field &field, const PossiblePass &pass, const Vector2 &position, const Line &blockLine);
    DefenderBot createBlockPass(const Field &field, PossiblePass &pass, const Vector2 &blockPoint);
    DefenderBot createBlockOnLine(const Field &field, const PossiblePass &pass, const Vector2 &blockPos);

    std::shared_ptr<Line> blockToGoalLine(const Field &field, const PossiblePass &pass);
    std::shared_ptr<Line> blockBallLine(const Field &field);
    std::shared_ptr<Vector2> blockOnDefenseAreaLine(const Field &field, const PossiblePass &pass);
    std::shared_ptr<Line> getBlockLineSegment(const Field &field, const Line &openGoalSegment, const Vector2 &point,
                                              double collisionRadius = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS(), double margin = -1.0);
    std::shared_ptr<Vector2> blockOnDefenseLine(const Field &field, const Line &openGoalSegment, const Vector2 &point);
    Vector2 getBlockPoint(const Line &openGoalSegment, const Vector2 &point, double collisionRadius);
    Line shortenLineForDefenseArea(const Field &field, const Vector2 &lineStart, const Vector2 &lineEnd, double defenseMargin);
    Vector2 getPosOnLine(const Line &line, double aggressionFactor);
    double getOrientation(const Line &line);
    Vector2 findPositionForBlockBall(const Field &field, const Line &line);

    std::vector<DefenderBot> decidePositions(const Field &field, const std::vector<DefenderBot> &lockedDefenders, std::vector<int> freeRobots);
    std::tuple<bool, int, std::vector<int>> decideLockedPositions(const Field &field, const std::vector<DefenderBot> &lockedDefenders, std::vector<int> freeRobots);

   private:
    const double defenceLineMargin = 0.15;        // min distance the points are from defence area. Should atleast be robotradius large.
    const double calculationCollisionRad = 0.15;  // distance at which our own robots are considered to be colliding in our calculation (prevents robots from stacking up too much)

    // we keep simulated versions of the data in this coach
    std::vector<world_new::view::RobotView> simulatedRobotsUs;
    std::vector<world_new::view::RobotView> simulatedRobotsThem;
    std::optional<world_new::view::BallView> simulatedBall;

  const double searchPoints = 31.0;  // amount of points we search for when we check if we can find points on a line
    std::vector<DefenderBot> defenders;

    std::vector<PossiblePass> createPassesSortedByDanger(const Field &field);
    std::vector<PossiblePass> sortPassesByDanger(std::vector<std::pair<PossiblePass, double>> &passesWithDanger);
    std::vector<std::pair<PossiblePass, double>> createPassesAndDanger(const Field &field);
    void removeBotFromWorld(int id, bool ourTeam);
    std::vector<world_new::view::RobotView> getTheirAttackers(const Field &field, std::vector<world_new::view::RobotView> them);

    bool validNewPosition(const Field &field, const Vector2 &position);
    std::shared_ptr<double> pickNewPosition(const Field &field, const Line &line);
    std::shared_ptr<Vector2> pickNewPosition(const Field &field, PossiblePass pass);

    void setupSimulatedWorld(const Field &field);

    std::shared_ptr<DefenderBot> blockMostDangerousPos(const Field &field);
    std::shared_ptr<DefenderBot> blockPass(const Field &field, PossiblePass pass);
    void addDefender(DefenderBot defender);
    void assignIDs(int lockedCount, std::vector<int> freeRobotIDs, const std::vector<DefenderBot> &oldDefenders);
    std::vector<Line> simulatedVisibleGoalParts(const Field &field, const PossiblePass &pass) const;
};
extern DefencePositionCoach g_defensivePositionCoach;

}  // namespace rtt::ai::coach

#endif  // ROBOTEAM_AI_DEFENSIVECOACH_H
