//
// Created by rolf on 18-10-18.
//

#include "WorldUtils.hpp"
namespace rtt{
namespace ai{
    std::vector<roboteam_msgs::WorldRobot> getAllBots(const roboteam_msgs::World& world) {
        std::vector<roboteam_msgs::WorldRobot> bots;
        bots.insert(bots.end(), world.us.begin(), world.us.end());
        bots.insert(bots.end(), world.them.begin(), world.them.end());
        return bots;
    }

    std::vector<TeamRobot> getAllTeamBots(const roboteam_msgs::World& world) {
        std::vector<TeamRobot> bots;
        for (const auto& bot : world.us) {
            bots.push_back({bot.id, true});
        }
        for (const auto& bot : world.them) {
            bots.push_back({bot.id, false});
        }
        return bots;
    }

    boost::optional<roboteam_msgs::WorldRobot> getWorldBot(unsigned int id, bool ourTeam, const roboteam_msgs::World& world) {
        const std::vector<roboteam_msgs::WorldRobot>& bots = ourTeam ? world.us : world.them;
        for (const auto& bot : bots) {
            if (bot.id == id) {
                return boost::optional<roboteam_msgs::WorldRobot>(bot);
            }
        }
        return boost::none;
    }

    boost::optional<TeamRobot> getTeamBot(unsigned int id, bool ourTeam, const roboteam_msgs::World& world) {
        std::vector<roboteam_msgs::WorldRobot> bots = ourTeam ? world.us : world.them;
        for (const auto& bot : bots) {
            if (bot.id == id) {
                return boost::optional<TeamRobot>({bot.id, ourTeam});
            }
        }
        return boost::none;
    }

}
}