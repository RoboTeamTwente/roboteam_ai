#pragma once

#include <vector>
#include <map>
#include "boost/optional.hpp"
#include "roboteam_msgs/WorldRobot.h"

namespace rtt {
namespace ai {
namespace dangerfinder {

typedef unsigned char DangerFlag;

/**
 * \struct DangerData
 * \brief Stores the output of DangerFinder
 */
struct DangerData {
  std::vector<int> dangerList;        //< A list of robot IDs, sorted from most to least dangerous
  std::map<int, double> scores;
  std::map<int, DangerFlag> flags;

  /**
   * \function getByDangerRank
   * \brief Gets a WorldRobot at the specified rank in this DangerData's dangerList, if it exists.
   */
  boost::optional<roboteam_msgs::WorldRobot> getByDangerRank(unsigned rank);
};

} // dangerfinder
} // ai
} // rtt
