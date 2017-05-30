#include "roboteam_world/danger_finder/DangerData.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_msgs/World.h"
#include "ros/ros.h"

namespace rtt {

boost::optional<roboteam_msgs::WorldRobot> getWorldBot(int id, bool ourTeam) {
	auto world = LastWorld::get();
	std::vector<roboteam_msgs::WorldRobot> vec = ourTeam ? world.us : world.them;
	for (const auto& bot : vec) {
		if (bot.id == id) return bot;
	}
	return boost::none;
}

boost::optional<roboteam_msgs::WorldRobot> DangerData::getByDangerRank(unsigned rank) {
	if (rank >= dangerList.size())
		return boost::none;
	int id = dangerList.at(rank);
	auto opt = getWorldBot(id, false);
	if (!opt) {
		ROS_WARN("DangerFinder: Opponent bot %d was in the dangerList, but could not be found by getWorldBot.", id);
	}
	return opt;
}

}
