/**
 * The dealer will check for the flags that are set in plays, but also for the distance
 * to a position that a robot might need to travel to. The lower the score of a robot, the better.
 */

// TODO Fix issue where roles get redistributed whilst robots are already in position
/// This issue occurs when there are multiple roles classes (defender+midfielder) that have the same priority

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
    std::unordered_map<std::string, v::RobotView> output;
    std::vector<std::pair<std::vector<double>, int>> scores = getScoreMatrix(allRobots, flagMap, stpInfoMap);
    // Make index of roles and ID to keep track which are the original indexes of each
    std::vector<int> indexRoles;
    std::vector<int> indexID;
    for (int i = 0; i < scores.size(); i++) {
        indexRoles.push_back(i);
        indexID.push_back(i);
    }

    // Get roles names
    std::vector<std::string> roleNames;
    for (auto const &[roleName, dealerFlags] : flagMap) {
        roleNames.push_back(roleName);
    }

    // Loop through the order of role priorities (column)
    for (int i = sizeof(DealerFlagPriority); i >= 0; i--){
        std::vector<std::vector<double>> currentScores; // Scores to be distributed (highest Priority)
        std::vector<int> newAssignments;                // Assignments from these scores
        std::vector<int> currentRoles;                  // Index of roles inside Score (role column)
        std::vector<int> originalRolesIndex;            // Index of roles from original index (role number)
        std::vector<int> currentIDs;                    // Index of ID's inside Score (robot row)
        std::vector<int> originalIDsIndex;              // Index of ID's from original index (robot id)

        // Check if a column has the looked for priorities
        for (int j = 0; j < scores.size(); j++){
            // if so, add it to a list and save the index
            if (scores.at(j).second == i) {
                currentScores.push_back(scores.at(j).first);    // get the score column
                currentRoles.push_back(j);                      // get the current index
                originalRolesIndex.push_back(indexRoles[j]);    // get the role number
            }
        }
        if (currentRoles.size() > 0) {
            // Return best assignment for those roles (column)
            rtt::Hungarian::Solve(currentScores, newAssignments);
            if (newAssignments.size() > 0) {
                for (int j = 0; j < newAssignments.size(); j++) {
                    if (newAssignments[j] >= 0) {
                        currentIDs.push_back(newAssignments[j]);                    // get newly assigned robot from current index
                        originalIDsIndex.push_back(indexID[currentIDs.back()]);     // get robot number
                        output.insert({roleNames[originalRolesIndex[j]], allRobots[originalIDsIndex.back()]});
                    }
                }
                if (output.size() == allRobots.size()) return output;               // case if there are less then 11 bots to distribute
                std::sort(currentIDs.begin(), currentIDs.end());                    // Sort to delete from back to front
                // Delete assigned roles and robots from score
                for (int j = currentRoles.size() - 1; j >= 0; j--) {
                    scores.erase(scores.begin() + currentRoles[j]);                 // remove role from score (col)
                    indexRoles.erase(indexRoles.begin() + currentRoles[j]);         // remove from index list
                }
                for (int col = 0; col < scores.back().first.size(); col++) {        // go through each score role (row)
                    for (int j = currentIDs.size() - 1; j >= 0; j--) {
                        scores[col].first.erase(scores[col].first.begin() + currentIDs[j]);     // remove the robot
                    }
                }
                for (int j = currentIDs.size() - 1; j >= 0; j--) {
                    indexID.erase(indexID.begin() + currentIDs[j]);                 // remove from index list
                }
            }
        }
    }
    return output;
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
            auto flagScore = scoreForFlags(dealerFlags.second, robot);              // [score,sum of weights]
            // Simple normalizer. DistanceScore has weight 1, the other factors can have various weights.
            row.push_back((distanceScore + flagScore.first)/(flagScore.second+1));  // the +1 is the distanceScore weight
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
        robotScore += ScoreForFlag.first;           // score
        totalFactor += ScoreForFlag.second;         // weight
    }
    return std::make_pair(robotScore,totalFactor);  // [score,sum of weights]
}

// Get the score of one flag for a role for one robot
std::pair <double, double> Dealer::getScoreForFlag(v::RobotView robot, Dealer::DealerFlag flag) {
    double factor = getFactorForPriority(flag.priority);
    return std::make_pair(factor * getDefaultFlagScores(robot, flag),factor); // [score,weight]
}

// Get the distance score for a robot to a position when there is a position that role needs to go to
double Dealer::getScoreForDistance(const stp::StpInfo &stpInfo, const v::RobotView &robot) {
    double distance{};
    if (stpInfo.getPositionToMoveTo().has_value()) {
        distance = robot->getPos().dist(stpInfo.getPositionToMoveTo().value());
    } else if (robot->getId() == GameStateManager::getCurrentGameState().keeperId) {
        distance = 0;
    } else if (stpInfo.getEnemyRobot().has_value()) {
        distance = robot->getPos().dist(stpInfo.getEnemyRobot().value()->getPos());
    }

    return costForDistance(distance, field->getFieldWidth(), field->getFieldLength());
}

// TODO these values need to be tuned.
double Dealer::getFactorForPriority(const DealerFlagPriority &flagPriority) {
    switch (flagPriority) {
        case DealerFlagPriority::LOW_PRIORITY:
            return 0.5;
        case DealerFlagPriority::MEDIUM_PRIORITY:
            return 1;
        case DealerFlagPriority::HIGH_PRIORITY:
            return 5;
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
