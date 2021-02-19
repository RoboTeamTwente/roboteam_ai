/**
 * The dealer will check for the flags that are set in plays, but also for the distance
 * to a position that a robot might need to travel to. The lower the score of a robot, the better.
 */
#include "utilities/Dealer.h"

#include <roboteam_utils/Hungarian.h>
#include <roboteam_utils/Print.h>

#include "utilities/GameStateManager.hpp"
#include "world/FieldComputations.h"

namespace rtt::ai {

Dealer::Dealer(v::WorldDataView world, rtt_world::Field *field) : world(world), field(field) {}

Dealer::DealerFlag::DealerFlag(DealerFlagTitle title, DealerFlagPriority priority) : title(title), priority(priority) {}

// Create a distribution of robots according to their flags
std::unordered_map<std::string, v::RobotView> Dealer::distribute(const std::vector<v::RobotView> &allRobots, const FlagMap &flagMap,
                                                                 const std::unordered_map<std::string, stp::StpInfo> &stpInfoMap) {
    RTT_DEBUG("STARTED")
    std::vector<std::pair<std::vector<double>, int>> scores = getScoreMatrix(allRobots, flagMap, stpInfoMap);
    std::vector<int> assignment(allRobots.size(),-1);
    // Loop through priorities
    // Delete given roles and robots from score
        RTT_DEBUG("MADE MATRIX")
        RTT_DEBUG("SIZE OF NEW ASSIGNMENTS = "+std::to_string(assignment.size()));
    // Loop through the order of role priorities (column)
    for (int i = sizeof(DealerFlagPriority); i >= 0; i--){
        RTT_DEBUG("----")
        RTT_DEBUG("STARTED ON "+std::to_string(i))
        std::vector<std::vector<double>> currentScores;
        std::vector<int> currentRoles;
        std::vector<int> currentAssignments;

        // Check if a column has the looked for priorities
        for (int j = 0; j < scores.size(); j++){
            // if so, add it to a list and save the index
            if (scores.at(j).second == i) {
                currentScores.push_back(scores.at(j).first);
                currentRoles.push_back(j);
                RTT_DEBUG("GIVING AWAY ROLE "+std::to_string(j))
            }
        }
        if (currentRoles.size() > 0) {
            RTT_DEBUG("# ROLES LOOKED AT " + std::to_string(currentRoles.size()))

            // Return best assignment for that roles (column)
            rtt::Hungarian::Solve(currentScores, currentAssignments);
            RTT_DEBUG("# ASSIGNMENTS LOOKED AT " + std::to_string(currentAssignments.size()))
            // find out what robots have been assigned and sort the list (this should be index of row's of score)
            vector<int> currentRobots;
            for (int i = 0; i < currentAssignments.size(); i++) {
                if (currentAssignments[i] != -1) {
                    currentRobots.push_back(currentAssignments[i]);
                    RTT_DEBUG("ROLE " + std::to_string(i) + " WAS GIVEN TO " + std::to_string(currentAssignments[i]))
                }
            }
            std::sort(currentRobots.begin(), currentRobots.end());
            // Delete assigned roles and robots from score
            for (int j = currentRoles.size() - 1; j >= 0; j--) {
                RTT_DEBUG("REMOVING ROLE " + std::to_string(currentRoles[j]))
                scores.erase(scores.begin() + currentRoles[j]);
            }
            RTT_DEBUG(" REMOVING ROBOTS FROM THE SCORE VECTORVECTOR")
            for (int col = 0; col < scores.size(); col++) {
                RTT_DEBUG(" COL " + std::to_string(col));
                for (int j = currentRobots.size() - 1; j >= 0; j--) {
                    scores[col].first.erase(scores[col].first.begin() + currentRobots[j]);
                }
            }
            for (int j = 0; j < assignment.size(); j++) {
                if (currentAssignments[j] != -1) assignment[j] = currentAssignments[j];
            }
        }
    }
    RTT_DEBUG("--- DONE ---");
    for (int i = 0; i < assignment.size(); i++){
        RTT_DEBUG(std::to_string(i)+ " given to "+std::to_string(assignment[i]))
    }
    return mapFromAssignments(allRobots, flagMap, assignment);
}

/* assignment now has the robot index in allRobots (not id) at the role index, and is ordered according to the roleNames
 * for example: assignment[0] = 2 // index
 * and roleNames[0] = role_1
 * robot_id = allRobots[index]
 * --> we can therefore make a map of <rolename, robot_id>
 */
std::unordered_map<std::string, v::RobotView> Dealer::mapFromAssignments(const std::vector<v::RobotView> &allRobots, const Dealer::FlagMap &flagMap,
                                                                         const std::vector<int> &assignment) const {
    vector<string> orderedRoleNames;
    for (auto const &[roleName, dealerFlags] : flagMap) {
        orderedRoleNames.push_back(roleName);
    }
    unordered_map<string, v::RobotView> result;
    for (int i = 0; i < orderedRoleNames.size(); i++) {
        if (assignment[i] != -1) {
            result.insert({orderedRoleNames[i], allRobots[assignment[i]]});
        }
    }
    return result;
}
// Populate a matrix with scores
std::vector<std::pair<std::vector<double>, int>> Dealer::getScoreMatrix(const std::vector<v::RobotView> &allRobots, const Dealer::FlagMap &flagMap,
                                                   const std::unordered_map<std::string, stp::StpInfo> &stpInfoMap) {
    std::vector<std::pair<std::vector<double>, int>> scores;
    scores.reserve(flagMap.size());

    // Loop through all roles that are in the dealerFlags map
    for (auto const &[roleName, dealerFlags] : flagMap) {
        std::vector<double> row;
        row.reserve(allRobots.size());

        // Calculate the score for each robot for a role; the row
        for (auto robot : allRobots) {
            double distanceScore{};

            if (stpInfoMap.find(roleName) != stpInfoMap.end()) {
                // The shorter the distance, the lower the distance score
                distanceScore = getScoreForDistance(stpInfoMap.find(roleName)->second, robot);
            }

            // The better the flags, the lower the score
            auto flagScore = scoreForFlags(dealerFlags.second, robot);
            row.push_back((distanceScore + flagScore.first)/(flagScore.second+1));
        }
        scores.emplace_back(row, (int) dealerFlags.first);
    }
    return scores;
}

// Calculate the score for all flags for a role for one robot
std::pair <double, double> Dealer::scoreForFlags(const std::vector<Dealer::DealerFlag> &dealerFlags, const v::RobotView &robot) {
    double robotScore = 0;
    double totalFactor = 0;
    for (auto flag : dealerFlags) {
        auto ScoreForFlag = getScoreForFlag(robot, flag);
        robotScore += ScoreForFlag.first;
        totalFactor += ScoreForFlag.second;
    }
    return std::make_pair(robotScore,totalFactor);
}

// Get the score of one flag for a role for one robot
std::pair <double, double> Dealer::getScoreForFlag(v::RobotView robot, Dealer::DealerFlag flag) {
    double factor = 1/getFactorForPriority(flag.priority);
    return std::make_pair(factor * getDefaultFlagScores(robot, flag),factor);
}

// Get the distance score for a robot to a position when there is a position that role needs to go to
double Dealer::getScoreForDistance(const stp::StpInfo &stpInfo, const v::RobotView &robot) {
    double distance{};
    if (stpInfo.getPositionToMoveTo().has_value()) {
        distance = robot->getPos().dist(stpInfo.getPositionToMoveTo().value());
    } else if (robot->getId() == GameStateManager::getCurrentGameState().keeperId) {
        distance = 0;
    } else if (stpInfo.getPositionToShootAt().has_value()) {
        distance = robot->getPos().dist(world.getBall()->get()->getPos());
    } else if (stpInfo.getEnemyRobot().has_value()) {
        distance = robot->getPos().dist(stpInfo.getEnemyRobot().value()->getPos());
    }

    return costForDistance(distance, field->getFieldWidth(), field->getFieldLength());
}

// TODO these values need to be tuned.
double Dealer::getFactorForPriority(const DealerFlagPriority &flagPriority) {
    switch (flagPriority) {
        case DealerFlagPriority::KEEPER:
            return 0.01;
        case DealerFlagPriority::LOW_PRIORITY:
            return 0.1;
        case DealerFlagPriority::MEDIUM_PRIORITY:
            return 1;
        case DealerFlagPriority::HIGH_PRIORITY:
            return 10;
        case DealerFlagPriority::REQUIRED:
            return 100;
        default:
            RTT_WARNING("Unhandled dealerflag!")
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
        case DealerFlagTitle::CLOSE_TO_BALL:
            return costForDistance(robot->getDistanceToBall(), fieldWidth, fieldLength);
        case DealerFlagTitle::CLOSE_TO_POSITIONING:
            return costForProperty(true);
        case DealerFlagTitle::WITH_WORKING_BALL_SENSOR:
            return costForProperty(robot->isWorkingBallSensor());
        case DealerFlagTitle::NOT_IMPORTANT:
            return costForProperty(false);
        case DealerFlagTitle::WITH_WORKING_DRIBBLER:
            return costForProperty(robot->isWorkingDribbler());
        case DealerFlagTitle::READY_TO_INTERCEPT_GOAL_SHOT: {
            // get distance to line between ball and goal
            // TODO this method can be improved by choosing a better line for the interception.
            LineSegment lineSegment = {world.getBall()->get()->getPos(), field->getOurGoalCenter()};
            return lineSegment.distanceToLine(robot->getPos());
        }
        case DealerFlagTitle::KEEPER:
            return costForProperty(robot->getId() == GameStateManager::getCurrentGameState().keeperId);
    }
    RTT_WARNING("Unhandled dealerflag!")
    return 0;
}

// Calculate the cost for distance. The further away the target, the higher the cost for that distance.
double Dealer::costForDistance(double distance, double fieldWidth, double fieldHeight) {
    auto fieldDiagonalLength = sqrt(pow(fieldWidth, 2.0) + pow(fieldHeight, 2.0));
    return distance / fieldDiagonalLength;
}

double Dealer::costForProperty(bool property) { return property ? 0.0 : 1.0; }

}  // namespace rtt::ai
