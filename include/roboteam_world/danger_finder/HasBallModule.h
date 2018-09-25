#pragma once

#include "roboteam_world/danger_finder/DangerModule.h"

namespace rtt {
namespace df {
/**
 * \class FreeModule
 * \brief DangerModule which checks whether one of their robots has the ball. Can set the DANGER_HAS_BALL flag.
 */
class HasBallModule : public DangerModule {
public:
	HasBallModule();
	PartialResult calculate(const roboteam_msgs::WorldRobot& bot, const roboteam_msgs::World& world) override;
};

}
}




