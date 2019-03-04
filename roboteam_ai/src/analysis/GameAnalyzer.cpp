//
// Created by mrlukasbos on 19-2-19.
//

#include <roboteam_ai/src/control/ControlUtils.h>
#include "GameAnalyzer.h"
#include "../utilities/World.h"
#include "../utilities/Field.h"

namespace rtt {
namespace ai {
namespace analysis {

playStyle GameAnalyzer::getRecommendedPlayStyle(bool ourTeam) {
    /*
     * If they are close
     * and have an immediate shot at goal
     */
    return DEFEND_WITH_ALL;
}


double GameAnalyzer::getBallPossessionEstimate(bool ourTeam) {
    double ballPossessionEstimate = 0.0;

    // booleans that matter
    bool teamHasBall = World::teamHasBall(ourTeam);
    bool teamHasShotAtGoal = true;
    bool teamHasAttackerAimedAtGoal = true;

    // doubles that matter
    double teamDistanceToGoal = 0;

    // integers that matter
    int amountOfOurRobots = 0;
    int amountOfTheirRobots = 0;

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

/// returns a danger score in a range from 0 - 100
double GameAnalyzer::evaluateRobotDangerScore(roboteam_msgs::WorldRobot robot, bool ourTeam) {
    double total = 0.0;

    Vector2 goalCenter = ourTeam ? Field::get_our_goal_center() : Field::get_their_goal_center();

    double distToGoal = Field::getDistanceToGoal(ourTeam, robot.pos);
    double shortestDistToEnemy = shortestDistToEnemyRobot(robot, ourTeam);
    double goalOpeningPercentage = Field::getPercentageOfGoalVisibleFromPoint(!ourTeam, robot.pos);

    bool isClosingIn = isClosingInToGoal(robot, ourTeam);
    bool canPass = !getRobotsToPassTo(robot, ourTeam).empty();
    bool hasBall = World::robotHasBall(robot, * World::getBall());
    bool aimedAtGoal = control::ControlUtils::robotIsAimedAtPoint(robot.id, ourTeam, goalCenter);
    bool closeToGoal = distToGoal < 10;
    bool standsFree = shortestDistToEnemy > 0.5;
    bool hasGoalOpening =  goalOpeningPercentage > 10;


    if (standsFree) total += 10;
    if (hasBall) total += 25;
    if (closeToGoal) total += 15;
    if (hasGoalOpening) total += 15;
    if (aimedAtGoal) total += 20;
    if (isClosingIn) total += 5;
    if (canPass) total += 10;

    return total;
}

double GameAnalyzer::robotCanShootAtGoalCertainty(roboteam_msgs::WorldRobot robot, bool ourTeam, roboteam_msgs::World simulatedWorld ) {

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

} // analysis
} // ai
} // rtt