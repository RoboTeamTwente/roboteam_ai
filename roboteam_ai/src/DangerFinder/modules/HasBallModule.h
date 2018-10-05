#pragma once

#include "DangerModule.h"

namespace rtt {
namespace ai {
namespace dangerfinder {

class HasBallModule : public DangerModule {
public:
	explicit HasBallModule(double danger);
	PartialResult calculate(const roboteam_msgs::WorldRobot& bot, const roboteam_msgs::World& world) override;
};

} // dangerfinder
} // ai
} // rtt





