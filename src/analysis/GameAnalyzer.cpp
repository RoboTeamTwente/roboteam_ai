#include "analysis/GameAnalyzer.h"
#include <control/ControlUtils.h>
#include <world/BallPossession.h>
#include "analysis/RobotDanger.h"
#include "world/FieldComputations.h"
#include "world_new/World.hpp"

namespace rtt::ai::analysis {

BallPossession GameAnalyzer::convertPossession(rtt::ai::BallPossession::Possession possession) {
    switch (possession) {
        default:
        case (rtt::ai::BallPossession::LOOSEBALL): {
            auto ballPosX = world_new::World::instance()->getWorld()->getBall().value()->getPos().x;
            if (ballPosX > 0) {
                return BallPossession::OFFENSIVE_NEUTRAL;
            } else {
                return BallPossession::DEFENSIVE_NEUTRAL;
            }
        }
        case (rtt::ai::BallPossession::CONTENDEDBALL):
            return BallPossession::NEUTRAL;
        case (rtt::ai::BallPossession::THEIRBALL):
            return BallPossession::THEY_HAVE_BALL;
        case (rtt::ai::BallPossession::OURBALL):
            return BallPossession::WE_HAVE_BALL;
    }
}

/// Get the average of the distances of robots to their opponents goal
double GameAnalyzer::getTeamDistanceToGoalAvg(const Field &field, bool ourGoal, std::vector<v::RobotView> robots) {
    double total = 0.0;
    for (auto robot : robots) {
        total += FieldComputations::getDistanceToGoal(field, ourGoal, robot->getPos());
    }
    return (total / robots.size());
}

/// returns a danger score
RobotDanger GameAnalyzer::evaluateRobotDangerScore(const Field &field, v::RobotView robot, bool ourTeam, v::WorldDataView world) {
    Vector2 goalCenter = ourTeam ? field.getOurGoalCenter() : field.getTheirGoalCenter();

    RobotDanger danger;
    danger.ourTeam = ourTeam;
    danger.id = robot->getId();
    danger.distanceToGoal = FieldComputations::getDistanceToGoal(field, ourTeam, robot->getPos());
    danger.shortestDistToEnemy = shortestDistToEnemyRobot(robot, ourTeam, world);
    danger.goalVisionPercentage = FieldComputations::getPercentageOfGoalVisibleFromPoint(field, !ourTeam, robot->getPos(), world);
    danger.robotsToPassTo = getRobotsToPassTo(robot, ourTeam, world);
    danger.aimedAtGoal = control::ControlUtils::robotIsAimedAtPoint(robot->getId(), ourTeam, goalCenter, world);

    return danger;
}

/// Check with distanceToLineWithEnds if there are obstructions
/// Returns all robots that can be passed to, along with the distance
/// we return robot ids instead of robot object because the objects are incorrect (because of simulated world)
std::vector<std::pair<int, double>> GameAnalyzer::getRobotsToPassTo(v::RobotView robot, bool ourTeam, v::WorldDataView world) {
    auto ourRobots = ourTeam ? world.getUs() : world.getThem();
    auto enemyRobots = ourTeam ? world.getThem() : world.getUs();

    std::vector<std::pair<int, double>> robotsToPassTo;
    for (auto ourRobot : ourRobots) {
        bool canPassToThisRobot = true;
        for (auto theirRobot : enemyRobots) {
            auto distToLine = control::ControlUtils::distanceToLineWithEnds(theirRobot->getPos(), Vector2(robot->getPos()), Vector2(ourRobot->getPos()));
            if (distToLine < (Constants::ROBOT_RADIUS_MAX() + Constants::BALL_RADIUS())) {
                canPassToThisRobot = false;
                break;
            }
        }
        if (canPassToThisRobot) {
            double distToRobot = (Vector2(ourRobot->getPos()) - Vector2(robot->getPos())).length();
            robotsToPassTo.emplace_back(std::make_pair(ourRobot->getId(), distToRobot));
        }
    }
    return robotsToPassTo;
}

/// get the shortest distance to an enemy robot
/// this is useful to check if a robot stands free
double GameAnalyzer::shortestDistToEnemyRobot(v::RobotView robot, bool ourTeam, v::WorldDataView world) {
    auto enemyRobots = ourTeam ? world.getThem() : world.getThem();
    Vector2 robotPos = robot->getPos();
    double shortestDist = INT_MAX;
    for (auto opponent : enemyRobots) {
        shortestDist = std::min(robotPos.dist(opponent->getPos()), shortestDist);
    }
    return shortestDist;
}

std::vector<std::pair<v::RobotView, RobotDanger>> GameAnalyzer::getRobotsSortedOnDanger(const Field &field, bool ourTeam, v::WorldDataView world) {
    auto robots = ourTeam ? world.getUs() : world.getThem();
    std::vector<std::pair<v::RobotView, RobotDanger>> robotDangers;

    for (auto robot : robots) {
        robotDangers.emplace_back(robot, evaluateRobotDangerScore(field, robot, ourTeam, world));
    }

    std::sort(robotDangers.begin(), robotDangers.end(),
              [&field](std::pair<v::RobotView, RobotDanger> a, std::pair<v::RobotView, RobotDanger> b) { return a.second.getTotalDanger(field) > b.second.getTotalDanger(field); });

    return robotDangers;
}

}  // namespace rtt::ai::analysis