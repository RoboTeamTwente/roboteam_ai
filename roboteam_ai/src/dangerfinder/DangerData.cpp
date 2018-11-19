#include "DangerData.h"
#include "ros/ros.h"

namespace rtt {
namespace ai {
namespace dangerfinder {

boost::optional<roboteam_msgs::WorldRobot> getWorldBot(int id, bool ourTeam) {
    auto world = rtt::ai::World::get_world();

    std::vector<roboteam_msgs::WorldRobot> vec = ourTeam ? world.us : world.them;
    for (const auto &bot : vec) {
        if (bot.id == (unsigned) id) return bot;
    }
    return boost::none;
}

/*
 * \function getByDangerRank
 * \brief Gets a WorldRobot at the specified rank in this DangerData's dangerList, if it exists.
 */
boost::optional<roboteam_msgs::WorldRobot> DangerData::getByDangerRank(unsigned rank) {
    if (rank >= dangerList.size())
        return boost::none;
    int id = dangerList.at(rank);
    auto opt = getWorldBot(id, false);
    if (! opt) {
        ROS_WARN(
                "dangerfinder: Opponent bot %d was in the dangerList, but could not be found by getWorldBot.",
                id);
    }
    return opt;
}

} // dangerfinder
} // ai
} // rtt