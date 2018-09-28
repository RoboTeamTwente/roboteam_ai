#pragma once

#include "DangerModule.h"

namespace rtt {
namespace df {

/**
 * \class OrientationModule
 * \brief DangerModule which assigns higher scores to robots which are oriented towards our goal.
 */
class OrientationModule final : public DangerModule {
public:
	OrientationModule(double factor = cfg().getConfigFor("Orientation").doubles["defaultDivisor"]);
	PartialResult calculate(const roboteam_msgs::WorldRobot& bot, const roboteam_msgs::World& world) override;
private:
	const double factor;
};

}
}
