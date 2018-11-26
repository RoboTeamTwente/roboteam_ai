#include "DangerData.h"
#include "ros/ros.h"

namespace rtt {
namespace ai {
namespace dangerfinder {

/*
 * \function getByDangerRank
 * \brief Gets a WorldRobot at the specified rank in this DangerData's dangerList, if it exists.
 */
boost::optional<roboteam_msgs::WorldRobot> DangerData::getByDangerRank(unsigned rank) {
    if (rank >= dangerList.size())
        return boost::none;
    int id = dangerList.at(rank);
    auto opt = World::getRobotForId(id, false);
    if (! opt) {
        ROS_WARN("dangerfinder: Opponent bot %d was in the dangerList, but could not be found in the world.", id);
    }
    return opt;
}

} // dangerfinder
} // ai
} // rtt