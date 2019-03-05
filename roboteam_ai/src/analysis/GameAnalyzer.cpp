//
// Created by mrlukasbos on 19-2-19.
//

#include <roboteam_ai/src/control/ControlUtils.h>
#include "GameAnalyzer.h"
#include "../utilities/World.h"
#include "../utilities/Field.h"
#include "RobotDanger.h"

namespace rtt {
namespace ai {
namespace analysis {

GameAnalyzer::GameAnalyzer() : running(false), stopping(false) { }

GameAnalyzer& GameAnalyzer::getInstance() {
    static GameAnalyzer instance;
    return instance;
}

AnalysisReport GameAnalyzer::getMostRecentReport() {
    return mostRecentReport;
}

AnalysisReport GameAnalyzer::generateReportNow() {
    AnalysisReport report;

    std::cout << "generating report" << std::endl;

    report.recommendedPlayStyle = getRecommendedPlayStyle();
    report.ballPossession = getBallPossessionEstimate(true);
    report.ourDistanceToGoalAvg = getTeamDistanceToGoalAvg(true);
    report.theirDistanceToGoalAvg = getTeamDistanceToGoalAvg(false);
    report.robotSortedOnDanger =  getRobotsSortedOnDanger(false);

    mostRecentReport = report;
    return report;
}

playStyle GameAnalyzer::getRecommendedPlayStyle() {
    return DEFEND_WITH_ALL;
}

double GameAnalyzer::getBallPossessionEstimate(bool ourTeam) {
    return 0;
}

/// Get the average of the distances of robots to their opponents goal
double GameAnalyzer::getTeamDistanceToGoalAvg(bool ourTeam, roboteam_msgs::World simulatedWorld ) {
    auto robots = ourTeam ? simulatedWorld.us : simulatedWorld.them;
    double total = 0.0;
    for (auto robot : robots) {
        total += Field::getDistanceToGoal(ourTeam, robot.pos);
    }
    return (total / robots.size());
}

/// return the attackers of a given team sorted on their vision on their opponents goal
std::vector<std::pair<roboteam_msgs::WorldRobot, double>> GameAnalyzer::getAttackersSortedOnGoalVision(bool ourTeam, roboteam_msgs::World simulatedWorld ) {
    auto robots = ourTeam ? simulatedWorld.us : simulatedWorld.them;
    std::vector<std::pair<roboteam_msgs::WorldRobot, double>> robotsWithVisibilities;

    for (auto robot : robots) {
        robotsWithVisibilities.emplace_back(robot, Field::getPercentageOfGoalVisibleFromPoint(!ourTeam, robot.pos));
    }

    // sort on goal visibility
    std::sort(robotsWithVisibilities.begin(), robotsWithVisibilities.end(),
            [](std::pair<roboteam_msgs::WorldRobot, double> a, std::pair<roboteam_msgs::WorldRobot, double> b){
       return  a.second < b.second;
    });

    return robotsWithVisibilities;
}

/// return the average goal vision of a given team towards their opponents goal
double GameAnalyzer::getTeamGoalVisionAvg(bool ourTeam, roboteam_msgs::World simulatedWorld ) {
    auto robots = ourTeam ? simulatedWorld.us : simulatedWorld.them;
    double total = 0.0;
    for (auto robot : robots) {
        total += Field::getPercentageOfGoalVisibleFromPoint(!ourTeam, robot.pos);
    }
    return (total/robots.size());
}

/// returns a danger score
RobotDanger GameAnalyzer::evaluateRobotDangerScore(roboteam_msgs::WorldRobot robot, bool ourTeam) {
    Vector2 goalCenter = ourTeam ? Field::get_our_goal_center() : Field::get_their_goal_center();

    RobotDanger danger;
    danger.ourTeam = ourTeam;
    danger.id = robot.id;
    danger.distanceToGoal = Field::getDistanceToGoal(ourTeam, robot.pos);
    danger.shortestDistToEnemy = shortestDistToEnemyRobot(robot, ourTeam);
    danger.goalVisionPercentage = Field::getPercentageOfGoalVisibleFromPoint(!ourTeam, robot.pos);
    danger.robotsToPassTo = getRobotsToPassTo(robot, ourTeam);
    danger.closingInToGoal = isClosingInToGoal(robot, ourTeam);
    danger.aimedAtGoal = control::ControlUtils::robotIsAimedAtPoint(robot.id, ourTeam, goalCenter);

    return danger;
}


/// Check with distanceToLineWithEnds if there are obstructions
/// Returns all robots that can be passed to, along with the distance
/// we return robot ids instead of robot object because the objects are incorrect (because of simulated world)
vector<pair<int, double>> GameAnalyzer::getRobotsToPassTo(roboteam_msgs::WorldRobot robot, bool ourTeam, roboteam_msgs::World simulatedWorld ) {
    auto ourRobots = ourTeam ? simulatedWorld.us : simulatedWorld.them;
    auto enemyRobots = ourTeam ? simulatedWorld.them : simulatedWorld.us;
    vector<pair<int, double>> robotsToPassTo;
    for (auto ourRobot : ourRobots) {
        bool canPassToThisRobot = true;
        for (auto theirRobot : enemyRobots) {
             auto distToLine = control::ControlUtils::distanceToLineWithEnds(theirRobot.pos, Vector2(robot.pos), Vector2(ourRobot.pos));
             if (distToLine < (Constants::ROBOT_RADIUS_MAX() + Constants::BALL_RADIUS())) {
                 canPassToThisRobot = false;
                 break;
             }
         }
         if (canPassToThisRobot) {
             double distToRobot = (Vector2(ourRobot.pos) - Vector2(robot.pos)).length();
             robotsToPassTo.emplace_back(make_pair(robot.id, distToRobot));
         }
    }
    return robotsToPassTo;
}

/// get the shortest distance to an enemy robot
/// this is useful to check if a robot stands free
double GameAnalyzer::shortestDistToEnemyRobot(roboteam_msgs::WorldRobot robot, bool ourTeam, roboteam_msgs::World simulatedWorld ) {
    auto enemyRobots = ourTeam ? simulatedWorld.them : simulatedWorld.us;
    Vector2 robotPos = robot.pos;
    double shortestDist = INT_MAX;
    for (auto opponent : enemyRobots) {
        shortestDist = std::min(robotPos.dist(opponent.pos), shortestDist);
    }
    return shortestDist;
}


/// check if a robot is closing in to our goal.
bool GameAnalyzer::isClosingInToGoal(roboteam_msgs::WorldRobot robot, bool ourTeam) {
    double distanceToGoal = Field::getDistanceToGoal(ourTeam, robot.pos);

    roboteam_msgs::World futureWorld = World::futureWorld(0.2);
    auto enemyRobots = ourTeam ? futureWorld.them : futureWorld.us;

    // find the robot in the future and look if it is closer than before
    for (auto futureRobot : enemyRobots) {
        if (futureRobot.id == robot.id && Field::getDistanceToGoal(ourTeam, futureRobot.pos) < distanceToGoal) {
            return true;
        }
    }
    return false;
}


void GameAnalyzer::start(int iterationsPerSecond) {
    if (!running) {
        ROS_INFO_STREAM_NAMED("GameAnalyzer", "Starting at " << iterationsPerSecond << " iterations per second");
        auto delay = (unsigned) (1000/iterationsPerSecond);
        thread = std::thread(&GameAnalyzer::loop, this, delay);
        running = true;
    }
}

// Stops the background worker thread.
void GameAnalyzer::stop() {
    if (running) {
        ROS_INFO_STREAM_NAMED("GameAnalyzer", "Stopping GameAnalyzer");
        thread.join();
        running = false;
    } else {
        ROS_INFO_STREAM_NAMED("GameAnalyzer", "Could not stop since it was not running in the first place.");
    }
}

void GameAnalyzer::loop(unsigned delayMillis) {
    std::chrono::milliseconds delay(delayMillis);
    while (!stopping) {
        generateReportNow();
        std::this_thread::sleep_for(delay);
    }
}

std::vector<std::pair<roboteam_msgs::WorldRobot, RobotDanger>> GameAnalyzer::getRobotsSortedOnDanger(bool ourTeam) {
    auto robots = ourTeam ? World::get_world().us : World::get_world().them;
    std::vector<std::pair<roboteam_msgs::WorldRobot, RobotDanger>> robotDangers;

    for (auto robot : robots) {
        robotDangers.emplace_back(std::make_pair(robot, evaluateRobotDangerScore(robot, ourTeam)));
    }

    std::sort(robotDangers.begin(), robotDangers.end(),
            [](std::pair<roboteam_msgs::WorldRobot, RobotDanger> a,
                    std::pair<roboteam_msgs::WorldRobot, RobotDanger> b) {
        return a.second.getTotalDanger() > b.second.getTotalDanger();
    });

    return robotDangers;
}

} // analysis
} // ai
} // rtt