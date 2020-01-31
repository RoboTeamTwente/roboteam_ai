//
// Created by mrlukasbos on 19-2-19.
//

#include "analysis/GameAnalyzer.h"
#include <control/ControlUtils.h>
#include <world/BallPossession.h>
#include "analysis/RobotDanger.h"
#include "world/FieldComputations.h"
#include "world/Robot.h"
#include "world/World.h"
#include "utilities/IOManager.h"

namespace rtt::ai::analysis {

GameAnalyzer::GameAnalyzer() : running(false), stopping(false) {}

GameAnalyzer &GameAnalyzer::getInstance() {
    static GameAnalyzer instance;
    return instance;
}

/// Generate a report with the game analysis
std::shared_ptr<AnalysisReport> GameAnalyzer::generateReportNow(const Field &field) {
    if (world::world->weHaveRobots()) {
        std::shared_ptr<AnalysisReport> report = std::make_shared<AnalysisReport>();

        report->ballPossession = convertPossession(ballPossessionPtr->getPossession());
        report->ourDistanceToGoalAvg = getTeamDistanceToGoalAvg(field, true);
        report->theirDistanceToGoalAvg = getTeamDistanceToGoalAvg(field, false);
        report->theirRobotSortedOnDanger = getRobotsSortedOnDanger(field, false);
        report->ourRobotsSortedOnDanger = getRobotsSortedOnDanger(field, true);

        std::lock_guard<std::mutex> lock(mutex);
        mostRecentReport = report;
        return report;
    }
    std::cout << "NOT A WORLD YET" << std::endl;
    start(30);
    return {};
}

BallPossession GameAnalyzer::convertPossession(rtt::ai::BallPossession::Possession possession) {
    switch (possession) {
        default:
        case (rtt::ai::BallPossession::LOOSEBALL): {
            auto ballPosX = rtt::ai::world::world->getBall()->getPos().x;
            if (ballPosX > 0) {
                return BallPossession::OFFENSIVE_NEUTRAL;
            } else {
                return BallPossession::DEFENSIVE_NEUTRAL;
            }
        }
        case (rtt::ai::BallPossession::CONTENDEDBALL):return BallPossession::NEUTRAL;
        case (rtt::ai::BallPossession::THEIRBALL):return BallPossession::THEY_HAVE_BALL;
        case (rtt::ai::BallPossession::OURBALL):return BallPossession::WE_HAVE_BALL;
    }
}

/// Get the average of the distances of robots to their opponents goal
double GameAnalyzer::getTeamDistanceToGoalAvg(const Field &field, bool ourTeam, WorldData simulatedWorld) {
    auto robots = ourTeam ? simulatedWorld.us : simulatedWorld.them;
    double total = 0.0;
    for (auto robot : robots) {
        total += FieldComputations::getDistanceToGoal(field, ourTeam, robot->pos);
    }
    return (total / robots.size());
}

/// returns a danger score
RobotDanger GameAnalyzer::evaluateRobotDangerScore(const Field &field, RobotPtr robot, bool ourTeam) {
    Vector2 goalCenter = ourTeam ? field.getOurGoalCenter() : field.getTheirGoalCenter();

    RobotDanger danger;
    danger.ourTeam = ourTeam;
    danger.id = robot->id;
    danger.distanceToGoal = FieldComputations::getDistanceToGoal(field, ourTeam, robot->pos);
    danger.shortestDistToEnemy = shortestDistToEnemyRobot(robot, ourTeam);
    danger.goalVisionPercentage = FieldComputations::getPercentageOfGoalVisibleFromPoint(field, !ourTeam, robot->pos,
                                                                                         world::world->getWorld());
    danger.robotsToPassTo = getRobotsToPassTo(robot, ourTeam);
    danger.closingInToGoal = isClosingInToGoal(field, robot, ourTeam);
    danger.aimedAtGoal = control::ControlUtils::robotIsAimedAtPoint(robot->id, ourTeam, goalCenter);

    return danger;
}

std::shared_ptr<AnalysisReport> GameAnalyzer::getMostRecentReport() {
    std::lock_guard<std::mutex> lock(mutex);
    return mostRecentReport;
}

/// Check with distanceToLineWithEnds if there are obstructions
/// Returns all robots that can be passed to, along with the distance
/// we return robot ids instead of robot object because the objects are incorrect (because of simulated world)
std::vector<std::pair<int, double>> GameAnalyzer::getRobotsToPassTo(RobotPtr robot, bool ourTeam, WorldData simulatedWorld) {
    auto ourRobots = ourTeam ? simulatedWorld.us : simulatedWorld.them;
    auto enemyRobots = ourTeam ? simulatedWorld.them : simulatedWorld.us;

    std::vector<std::pair<int, double>> robotsToPassTo;
    for (auto ourRobot : ourRobots) {
        bool canPassToThisRobot = true;
        for (auto theirRobot : enemyRobots) {
            auto distToLine = control::ControlUtils::distanceToLineWithEnds(theirRobot->pos, Vector2(robot->pos),
                                                                            Vector2(ourRobot->pos));
            if (distToLine < (Constants::ROBOT_RADIUS_MAX() + Constants::BALL_RADIUS())) {
                canPassToThisRobot = false;
                break;
            }
        }
        if (canPassToThisRobot) {
            double distToRobot = (Vector2(ourRobot->pos) - Vector2(robot->pos)).length();
            robotsToPassTo.emplace_back(std::make_pair(ourRobot->id, distToRobot));
        }
    }
    return robotsToPassTo;
}

/// get the shortest distance to an enemy robot
/// this is useful to check if a robot stands free
double GameAnalyzer::shortestDistToEnemyRobot(RobotPtr robot, bool ourTeam, WorldData simulatedWorld) {
    auto enemyRobots = ourTeam ? simulatedWorld.them : simulatedWorld.us;
    Vector2 robotPos = robot->pos;
    double shortestDist = INT_MAX;
    for (auto opponent : enemyRobots) {
        shortestDist = std::min(robotPos.dist(opponent->pos), shortestDist);
    }
    return shortestDist;
}

/// check if a robot is closing in to our goal.
bool GameAnalyzer::isClosingInToGoal(const Field &field, RobotPtr robot, bool ourTeam) {
    double distanceToGoal = FieldComputations::getDistanceToGoal(field, ourTeam, robot->pos);

    WorldData futureWorld = world::world->getFutureWorld(0.2);
    auto enemyRobots = ourTeam ? futureWorld.them : futureWorld.us;

    // find the robot in the future and look if it is closer than before
    for (auto futureRobot : enemyRobots) {
        if (futureRobot->id == robot->id &&
            FieldComputations::getDistanceToGoal(field, ourTeam, futureRobot->pos) < distanceToGoal) {
            return true;
        }
    }
    return false;
}

void GameAnalyzer::start(int iterationsPerSecond) {
    if (!running && world::world->weHaveRobots()) {
        std::cout << "GameAnalyzer: " << "Starting at " << iterationsPerSecond << " iterations per second" << std::endl;
        auto delay = (unsigned) (1000.0 / iterationsPerSecond);
        thread = std::thread(&GameAnalyzer::loop, this, delay);
        running = true;
    }
}

// Stops the background worker thread.
void GameAnalyzer::stop() {
    stopping = true;
    if (running || stopping) {
        std::cout << "GameAnalyzer: " << "Stopping GameAnalyzer" << std::endl;
        thread.join();
        running = false;
        stopping = false;
    } else {
        std::cout << "GameAnalyzer: " << "Could not stop since it was not running in the first place." << std::endl;
    }
}

void GameAnalyzer::loop(unsigned delayMillis) {
    std::chrono::milliseconds delay(delayMillis);
    while (!stopping) {
        const Field &field = io::io.getField();
        generateReportNow(field);
        std::this_thread::sleep_for(delay);
    }
}

std::vector<std::pair<GameAnalyzer::RobotPtr, RobotDanger>> GameAnalyzer::getRobotsSortedOnDanger(const Field &field,
                                                                                                  bool ourTeam) {
    auto robots = ourTeam ? world::world->getUs() : world::world->getThem();
    std::vector<std::pair<RobotPtr, RobotDanger>> robotDangers;

    for (auto robot : robots) {
        robotDangers.emplace_back(robot, evaluateRobotDangerScore(field, robot, ourTeam));
    }

    std::sort(robotDangers.begin(), robotDangers.end(),
              [&field](std::pair<RobotPtr, RobotDanger> a, std::pair<RobotPtr, RobotDanger> b) {
                return a.second.getTotalDanger(field) > b.second.getTotalDanger(field);
              });

    return robotDangers;
}

}  // namespace rtt::ai::analysis