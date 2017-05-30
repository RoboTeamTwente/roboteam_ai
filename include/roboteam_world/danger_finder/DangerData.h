#pragma once

#include <vector>
#include <map>
#include "boost/optional.hpp"
#include "roboteam_msgs/WorldRobot.h"

namespace rtt {

typedef unsigned char DangerFlag;

/**
 * \struct DangerData
 * \brief Stores the output of DangerFinder
 */
struct DangerData {
	std::vector<int> dangerList;		//< A list of robot IDs, sorted from most to least dangerous
	std::map<int, double> scores;
	std::map<int, DangerFlag> flags;

	boost::optional<roboteam_msgs::WorldRobot> getByDangerRank(unsigned rank);
};


}
