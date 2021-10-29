/**
 * The dealer will check for the flags that are set in plays, but also for the distance
 * to a position that a robot might need to travel to. The lower the score of a robot, the better.
 */

// TODO Fix issue where roles get redistributed whilst robots are already in position
/// This issue occurs when there are multiple roles classes (defender+midfielder) that have the same priority

#include "utilities/Dealer.h"
#include <iterator>

#include <roboteam_utils/Hungarian.h>
#include <roboteam_utils/Print.h>

#include "utilities/GameStateManager.hpp"
#include "world/FieldComputations.h"

namespace rtt::ai {

Dealer::Dealer(v::WorldDataView world, rtt_world::Field *field) : world(world), field(field) {}

Dealer::DealerFlag::DealerFlag(DealerFlagTitle title, DealerFlagPriority priority) : title(title), priority(priority) {}

// Create a distribution of robots according to their flags
std::unordered_map<std::string, v::RobotView> Dealer::distribute(std::vector<v::RobotView> allRobots, FlagMap flagMap,
                                                                 const std::unordered_map<std::string, stp::StpInfo> &stpInfoMap) {
    std::unordered_map<std::string, v::RobotView> output;
    // Remove all forcedID's before continuing computations
    distribute_forcedIDs(allRobots,flagMap,output);

    std::vector<RoleScores> scores = getScoreMatrix(allRobots, flagMap, stpInfoMap);
    // Make index of roles and ID to keep track which are the original indexes of each and get roleNames
    std::vector<int> indexRoles;
    std::vector<int> indexID;
    std::vector<std::string> roleNames;
    distribute_init(indexRoles,indexID,roleNames,flagMap);

    // Loop through the order of role priorities (column)
    for (const auto currentPriority : PriorityOrder){
        DealerDistribute current{};
        // Check if a column has the looked for priorities
        for (std::size_t j = 0; j < scores.size(); j++){
            // if so, add it to a list and save the index
            if (scores[j].priority == currentPriority) {
                current.currentScores.push_back(scores.at(j).robotScores);    // get the score column
                current.currentRoles.push_back(j);                      // get the current index
                current.originalRolesIndex.push_back(indexRoles[j]);    // get the role number
            }
        }
        if (!current.currentRoles.empty()) {
            // Return best assignment for those roles (column)
            rtt::Hungarian::Solve(current.currentScores, current.newAssignments);
            if (!current.newAssignments.empty()) {
                for (std::size_t j = 0; j < current.newAssignments.size(); j++) {
                    if (current.newAssignments[j] >= 0) {
                        current.currentIDs.push_back(current.newAssignments[j]);                    // get newly assigned robot from current index
                        current.originalIDsIndex.push_back(indexID[current.currentIDs.back()]);     // get robot number
                        output.insert({roleNames[current.originalRolesIndex[j]], allRobots[current.originalIDsIndex.back()]});
                    }
                }
                if (output.size() == allRobots.size()) return output;               // case if there are less then 11 bots to distribute
                distribute_remove(current,indexRoles,indexID,scores);
            }
        }
    }
    return output;
}

void Dealer::distribute_forcedIDs(std::vector<v::RobotView> &allRobots, FlagMap& flagMap, std::unordered_map<std::string, v::RobotView>& output){
    for (auto role = flagMap.begin(); role != flagMap.end(); ++role) {
        int ID = role->second.forcedID;
        if (ID != -1){
            // Check if that ID is a friendly ID
            if (!std::any_of(allRobots.begin(),allRobots.end(),[ID](v::RobotView& x){
                return x->getId() == ID;
            }) ) {
                RTT_ERROR("ID " + std::to_string(ID) + " is not a VALID ID. The force ID will be IGNORED.")
                continue;
            }
            output.insert({role->first,allRobots[ID]}); // Assign role to ID
            allRobots.erase(allRobots.begin() + ID);    // Remove the robot to reduce future computations
            flagMap.erase(role--);              // Remove role to reduce future computations
        }
    }
}

void Dealer::distribute_init(std::vector<int>& indexRoles, std::vector<int>& indexID, std::vector<std::string>& roleNames, const Dealer::FlagMap &flagMap) {
    indexRoles.reserve(flagMap.size());
    indexID.reserve(flagMap.size());
    roleNames.reserve(flagMap.size());
    for (std::size_t i = 0; i < flagMap.size(); i++) {
        indexRoles.push_back(i);
        indexID.push_back(i);
    }
    for (auto const &[roleName, dealerFlags] : flagMap) {
        roleNames.push_back(roleName);
    }
}

// Delete assigned roles and robots from score
void Dealer::distribute_remove(DealerDistribute& current, std::vector<int>& indexRoles, std::vector<int>& indexID, std::vector<RoleScores>& scores){
    std::sort(current.currentIDs.begin(), current.currentIDs.end());                    // Sort to delete from back to front

    for (auto i = current.currentRoles.rbegin(); i != current.currentRoles.rend(); ++i) {
        assert(*i < scores.size());
        assert(*i < indexRoles.size());
        scores.erase(scores.begin() + *i);                 // remove role from score (col)
        indexRoles.erase(indexRoles.begin() + *i);         // remove from index list
    }
    for (auto &i : scores) {        // go through each score role (row)
        for (auto j = current.currentIDs.rbegin(); j != current.currentIDs.rend(); ++j) {
            assert(*j < i.robotScores.size());
            i.robotScores.erase(i.robotScores.begin() + *j);     // remove the robot
        }
    }
    for (auto i = current.currentIDs.rbegin(); i != current.currentIDs.rend(); ++i) {
        assert(*i < indexID.size());
        indexID.erase(indexID.begin() + *i);                 // remove from index list
    }
}

// Populate a matrix with scores
std::vector<Dealer::RoleScores> Dealer::getScoreMatrix(const std::vector<v::RobotView> &allRobots, const Dealer::FlagMap &flagMap,
                                                   const std::unordered_map<std::string, stp::StpInfo> &stpInfoMap) {
    std::vector<RoleScores> scores;
    for (auto i : scores) i.robotScores.reserve(flagMap.size());
    // Loop through all roles that are in the dealerFlags map
    for (auto const &[roleName, dealerFlags] : flagMap) {
        std::vector<double> role;
        role.reserve(allRobots.size());
        // Calculate the score for each robot for a role; the row
        for (auto robot : allRobots) {
            double robotDistanceScore{};
            if (stpInfoMap.find(roleName) != stpInfoMap.end()) {
                // The shorter the distance, the lower the distance score
                robotDistanceScore = getRobotScoreForDistance(stpInfoMap.find(roleName)->second, robot);
            }
            // The better the flags, the lower the score
            auto robotScore = getRobotScoreForRole(dealerFlags.flags, robot);
            // Simple normalizer. DistanceScore has weight 1, the other factors can have various weights.
            role.push_back((robotDistanceScore + robotScore.sumScore) / (robotScore.sumWeights + 1));  // the +1 is the distanceScore weight
        }
        scores.push_back({role, dealerFlags.priority});
    }
    return scores;
}

// Calculate the score for all flags for a role for one robot
Dealer::RobotRoleScore Dealer::getRobotScoreForRole(const std::vector<Dealer::DealerFlag> &dealerFlags, const v::RobotView &robot) {
    double robotScore = 0;
    double sumWeights = 0;
    for (auto flag : dealerFlags) {
        FlagScore ScoreForFlag = getRobotScoreForFlag(robot, flag);
        robotScore += ScoreForFlag.score;
        sumWeights += ScoreForFlag.weight;
    }
    return {robotScore,sumWeights};  // [score,sum of weights]
}

// Get the score of one flag for a role for one robot
Dealer::FlagScore Dealer::getRobotScoreForFlag(v::RobotView robot, Dealer::DealerFlag flag) {
    double factor = getWeightForPriority(flag.priority);
    return {factor * getDefaultFlagScores(robot, flag),factor}; // [score,weight]
}

// Get the distance score for a robot to a position when there is a position that role needs to go to
double Dealer::getRobotScoreForDistance(const stp::StpInfo &stpInfo, const v::RobotView &robot) {
    double distance{};
    if (robot->getId() == GameStateManager::getCurrentGameState().keeperId) {
        distance = 0;
    } else if (stpInfo.getPositionToMoveTo().has_value()) {
        distance = robot->getPos().dist(stpInfo.getPositionToMoveTo().value());
    } else if (stpInfo.getEnemyRobot().has_value()) {
        distance = robot->getPos().dist(stpInfo.getEnemyRobot().value()->getPos());
    }

    return costForDistance(distance, field->getFieldWidth(), field->getFieldLength());
}

// TODO these values need to be tuned.
double Dealer::getWeightForPriority(const DealerFlagPriority &flagPriority) {
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
        case DealerFlagTitle::CLOSEST_TO_BALL:
            return costForProperty(robot->getId() == world.getRobotClosestToBall(rtt::world::us)->get()->getId());
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
