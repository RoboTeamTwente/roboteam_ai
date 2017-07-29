#pragma once

#include "roboteam_world/danger_finder/DangerModule.h"

namespace rtt {
namespace df {

class HasBallModule : public DangerModule {
public:
	HasBallModule();
	PartialResult calculate(const roboteam_msgs::WorldRobot& bot, const roboteam_msgs::World& world) override;
};

}
}




