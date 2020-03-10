#include <roboteam_utils/Hungarian.h>
#include "utilities/Dealer.h"
#include <roboteam_utils/LineSegment.h>
#include "world/FieldComputations.h"
#include <roboteam_utils/Print.h>

namespace rtt::ai {

Dealer::Dealer(v::WorldDataView world, world::Field * field)
    : world(world), field(field) { }

Dealer::DealerFlag::DealerFlag(DealerFlagTitle title, DealerFlagPriority priority)
    : title(title), priority(priority) {}

// Create a distribution of robots according to their flags
std::unordered_map<std::string, v::RobotView> Dealer::distribute(const std::vector<v::RobotView>& allRobots, const FlagMap &flagMap) {
    std::vector<std::vector<double>> scores = getScoreMatrix(allRobots, flagMap);
    std::vector<int> assignment;

    // solve the matrix and put the results in 'assignment'
    rtt::Hungarian::Solve(scores, assignment);

    return mapFromAssignments(allRobots, flagMap, assignment);
}

/* assignments now has the robot ids at the role index, and is ordered according to the roleNames
* for example: assignments[0] = 2 // robot_id
* and roleNames[0] = role_1
* --> we can therefore make a map of <rolename, robot_id>
*/
std::unordered_map<std::string, v::RobotView> Dealer::mapFromAssignments(const std::vector<v::RobotView> &allRobots,
                                                                  const Dealer::FlagMap &flagMap,
                                                                  const std::vector<int> &assignment) const {
    vector<string> orderedRoleNames;
    for (auto const& [roleName, dealerFlags] : flagMap) {
        orderedRoleNames.push_back(roleName);
    }
    unordered_map<string, v::RobotView> result;
    for (int i = 0; i < orderedRoleNames.size(); i++) {
        for (auto robot : allRobots) {
            if (robot->getId() == assignment[i]) {
                result.insert({orderedRoleNames[i], robot});
            }
        }
    }
    return result;
}

// Populate a matrix with scores
std::vector<vector<double>> Dealer::getScoreMatrix(const std::vector<v::RobotView> &allRobots, const Dealer::FlagMap &flagMap) {
    vector<vector<double>> scores;
    scores.reserve(flagMap.size());
    for (auto const& [roleName, dealerFlags] : flagMap) {
        std::vector<double> row;
        row.reserve(allRobots.size());
        for (auto robot : allRobots) {
            row.push_back(scoreForFlags(dealerFlags, robot));
        }
        scores.push_back(row);
    }
    return scores;
}

double Dealer::scoreForFlags(const std::vector<Dealer::DealerFlag> &dealerFlags, const v::RobotView &robot) {
    double robotScore = 0;
    for (auto flag : dealerFlags) {
        robotScore += getScoreForFlag(robot, flag);
    }
    return robotScore;
}

double Dealer::getScoreForFlag(v::RobotView robot, Dealer::DealerFlag flag) {
    double factor = getFactorForPriority(flag);
    return factor * getDefaultFlagScores(robot, flag);
}

// TODO these values need to be tuned.
double Dealer::getFactorForPriority(const Dealer::DealerFlag &flag) {
    switch (flag.priority) {
        case DealerFlagPriority::LOW_PRIORITY: return 1.0;
        case DealerFlagPriority::MEDIUM_PRIORITY: return 2.0;
        case DealerFlagPriority::HIGH_PRIORITY: return 3.0;
        default:
            std::cerr << "[Dealer] Unhandled dealerflag!" << endl;
            return 0;
    }
}

// TODO these values need to be tuned. 
double Dealer::getDefaultFlagScores(const v::RobotView &robot, const Dealer::DealerFlag &flag) {
    auto fieldWidth = field->getFieldWidth(); 
    auto fieldLength = field->getFieldLength(); 
    switch (flag.title) {
        case DealerFlagTitle::CLOSE_TO_THEIR_GOAL: 
            return costForDistance(FieldComputations::getDistanceToGoal(*field, false, robot->getPos()), fieldWidth, fieldLength);
        case DealerFlagTitle::CLOSE_TO_OUR_GOAL: 
            return costForDistance(FieldComputations::getDistanceToGoal(*field, true, robot->getPos()), fieldWidth, fieldLength);
        case DealerFlagTitle::CLOSE_TO_BALL: {
            auto ball = world.getBall();
            if (!ball) return 0.0;
            return robot->getPos().dist(ball.value()->getPos());
        }
        case DealerFlagTitle::WITH_WORKING_BALL_SENSOR: return costForProperty(robot->isWorkingBallSensor());
        case DealerFlagTitle::ROBOT_TYPE_50W: return costForProperty(robot->isFiftyWatt());
        case DealerFlagTitle::ROBOT_TYPE_30W: return costForProperty(robot->isThirtyWatt());
        case DealerFlagTitle::WITH_WORKING_DRIBBLER: return costForProperty(robot->isWorkingDribbler());
        case DealerFlagTitle::READY_TO_INTERCEPT_GOAL_SHOT: {
            // get distance to line between ball and goal
            // TODO this method can be improved by choosing a better line for the interception.
            LineSegment lineSegment = {world.getBall()->get()->getPos(), field->getOurGoalCenter()};
            return lineSegment.distanceToLine(robot->getPos());
        }
    }
    RTT_WARNING("Unhandled dealerflag!");
    return 0;
}

double Dealer::costForDistance(double distance, double fieldWidth, double fieldHeight) {
    auto fieldDiagonalLength = sqrt(pow(fieldWidth, 2.0) + pow(fieldHeight, 2.0));
    return distance/fieldDiagonalLength;
}

double Dealer::costForProperty(bool property) {
    return property ? 0.0 : 1.0;
}

} // rtt::ai
