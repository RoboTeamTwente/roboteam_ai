#pragma once

#include "roboteam_world/danger_finder/DangerModule.h"

namespace rtt {
namespace df {

/**
 * \class FreeModule
 * \brief DangerModule which checks whether a robot could potentially receive the ball.
 * If so, it gives it a score and sets the DANGER_FREE flag.
 */
class FreeModule : public DangerModule {
public:
	FreeModule();
	PartialResult calculate(const roboteam_msgs::WorldRobot& bot, const roboteam_msgs::World& world) override;
};

}
}
