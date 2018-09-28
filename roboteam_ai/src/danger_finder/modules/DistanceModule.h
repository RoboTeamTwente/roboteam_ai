#pragma once

#include "DangerModule.h"
#include <map>

namespace rtt {
namespace df {

/**
 * \class DistanceModule
 * \brief A DangerModule which assigns higher scores to robots which are closer to our goal.
 * It may also set the DANGER_CLOSING flag if applicable.
 */
class DistanceModule final : public DangerModule {
public:
	DistanceModule(double factor = cfg().getConfigFor("Distance").doubles["defaultDivisor"]);
	PartialResult calculate(const roboteam_msgs::WorldRobot& bot, const roboteam_msgs::World& world) override;
private:
	const double factor;
	std::map<int, double> lastDistances;
};

}
}
