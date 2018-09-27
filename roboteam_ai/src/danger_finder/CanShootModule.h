#pragma once

#pragma once

#include "DangerModule.h"

namespace rtt {
namespace df {

/**
 * DangerModule which checks whether a robot has the ball as well as line of sight to our goal.
 * If so, it gives it a massive danger score and sets the DANGER_CAN_SHOOT flag.
 */
class CanShootModule : public DangerModule {
public:
	CanShootModule();
	PartialResult calculate(const roboteam_msgs::WorldRobot& bot, const roboteam_msgs::World& world) override;
};

}
}

