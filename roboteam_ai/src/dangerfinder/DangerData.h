#ifndef ROBOTEAM_AI_DANGERDATA_H
#define ROBOTEAM_AI_DANGERDATA_H

#include <vector>
#include <map>
#include "boost/optional.hpp"
#include "roboteam_msgs/WorldRobot.h"
#include "../../src/utilities/World.h"

namespace rtt {
namespace ai {
namespace dangerfinder {

typedef unsigned char DangerFlag;

/**
 * \struct DangerData
 * \brief Stores the output of DangerFinder
 */
struct DangerData {
  std::vector<int> dangerList; // A list of robot IDs, sorted from most to least dangerous
  std::map<int, double> scores;
  std::map<int, DangerFlag> flags;

  boost::optional<roboteam_msgs::WorldRobot> getByDangerRank(unsigned rank);
};

} // dangerfinder
} // ai
} // rtt

#endif // ROBOTEAM_AI_DANGERDATA_H