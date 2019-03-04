//
// Created by mrlukasbos on 19-2-19.
//

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
double GameAnalyzer::getTeamDistanceToGoalAvg(bool ourTeam) {
    auto robots = ourTeam ? World::get_world().us : World::get_world().them;
    double total = 0.0;
    for (auto robot : robots) {
        total += Field::getDistanceToGoal(ourTeam, robot.pos);
    }
    return (total / robots.size());
}

/// return the attackers of a given team sorted on their vision on their opponents goal
std::vector<std::pair<roboteam_msgs::WorldRobot, double>> GameAnalyzer::getAttackersSortedOnGoalVision(bool ourTeam) {
    auto robots = ourTeam ? World::get_world().us : World::get_world().them;
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
double GameAnalyzer::getTeamGoalVisionAvg(bool ourTeam) {
    auto robots = ourTeam ? World::get_world().us : World::get_world().them;
    double total = 0.0;
    for (auto robot : robots) {
        total += Field::getPercentageOfGoalVisibleFromPoint(!ourTeam, robot.pos);
    }
    return (total/robots.size());
}

///
double GameAnalyzer::evaluateRobotDangerScore(roboteam_msgs::WorldRobot robot, bool ourTeam) {
    double total = 0.0;

    total += World::robotHasBall(robot, * World::getBall()) ? 100 : 0;

    total += std::max(10 - Field::getDistanceToGoal(ourTeam, robot.pos), 0.0);

    total += Field::getPercentageOfGoalVisibleFromPoint(!ourTeam, robot.pos);

    // check if a robot stands free, meaning the closest distance to it's enemy robots is larger than X
    total += shortestDistToEnemyRobot(robot, ourTeam) > 0.5;



    // hasBall
    // can pass
    // has goal vision
    // can shoot at goal
    // is close to goal
    // is closing in to goal
    // is standing free

    return total;
}

double GameAnalyzer::robotCanShootAtGoalCertainty(roboteam_msgs::WorldRobot robot, bool ourTeam) {

}

int GameAnalyzer::getRobotsToPassTo(roboteam_msgs::WorldRobot robot, bool ourTeam) {

    // get robots from this team
    // get robots from opponent team

    // draw line
    return 0;
}

double GameAnalyzer::shortestDistToEnemyRobot(roboteam_msgs::WorldRobot robot, bool ourTeam) {
    auto enemyRobots = ourTeam ? World::get_world().us : World::get_world().them;
    Vector2 robotPos = robot.pos;
    double shortestDist = INT_MAX;
    for (auto opponent : enemyRobots) {
        shortestDist = std::min(robotPos.dist(opponent.pos), shortestDist);
    }
    return shortestDist;
}

} // analysis
} // ai
} // rtt