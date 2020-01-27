#include "include/roboteam_ai/utilities/Dealer.h"

namespace rtt::ai {

Dealer::DealerFlag::DealerFlag(DealerFlagTitle title, bool important)
    : title(std::move(title)), important(important) {}

Dealer::Role::Role(std::string roleName, int robot) : name(std::move(roleName)), robotId(robot) {}

std::optional<int> Dealer::claimRobot(std::vector<int> allRobots, const std::string& roleName, std::vector<DealerFlag> flags, bool allowShuffle) {
    if (claimedRoles.count(roleName) == 0) {
        auto robot = getOptimalRobot(std::move(allRobots), {});
        if (robot) {
            auto newRole = Role(roleName, robot.value());
            claimedRoles.insert({roleName, newRole});
        } else {
            std::cerr << "[Dealer] Could not find optimal robot for role: " << roleName << std::endl;
            return std::nullopt;
        }
    }
    std::cout << "[Dealer] Robot was already claimed for role " << roleName << std::endl;
    return std::nullopt;
}

void Dealer::freeRobot(const std::string& roleName) {
    if (claimedRoles.count(roleName) == 0) {
        std::cout << "[Dealer] Robot was already free for role: " << roleName << std::endl;
        return;
    } else {
        claimedRoles.erase(roleName);
    }
}

std::vector<Dealer::Role> Dealer::getClaimedRoles() {
    std::vector<Role> vec;
    for (const auto& pair : claimedRoles) {
        vec.push_back(pair.second);
    }
    return vec;
}

std::vector<int> Dealer::getClaimedRobotIds() {
    std::vector<int> claimedRobotIds;
    for (const auto& pair : claimedRoles) {
        claimedRobotIds.push_back(pair.second.robotId);
    }
    return claimedRobotIds;
}

void Dealer::freeAllRobots() {
    claimedRoles.clear();
}

void Dealer::reshuffle() {

}

// Determine the set of free robots from all available robots and the claimed robots.
std::vector<int> Dealer::getFreeRobots(std::vector<int> allRobots, std::vector<int> claimedRobots) {
    std::vector<int> freeRobots;
    std::set_difference(allRobots.begin(), allRobots.end(), claimedRobots.begin(), claimedRobots.end(),
                        std::inserter(freeRobots, freeRobots.begin()));
    return freeRobots;
}

std::optional<int> Dealer::getOptimalRobot(std::vector<int> allRobots, const std::unordered_map<std::string, std::vector<DealerFlag>>& flagMap) {
    // populate a matrix with scores
    std::vector<std::vector<double>> scores;

    for (int i = 0; i < allRobots.size(); i++) {
        auto robot = allRobots.at(i);

        for (auto flagPair : flagMap) {
            auto flagPair = flagMap.at(j);
            double robotScore = 0;

            // the list of flag enums is in second
            for (auto flag : flagPair.second) {
                robotScore += getRobotScore(robot, flag);
            }
            scores[i][j] = robotScore;
        }
    }

    if (allRobots.empty())  return std::nullopt;
    return std::optional<int>(allRobots.at(0));
}



int Dealer::getScoreForFlag(int robotId, Dealer::DealerFlag flag) {
    return 10;
}

} // rtt::ai