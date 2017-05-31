#include "roboteam_world/danger_finder/CanShootModule.h"
#include "roboteam_utils/world_analysis.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/LastWorld.h"

namespace rtt {
namespace df {

CanShootModule::CanShootModule() : DangerModule("CanShoot") {}

PartialResult CanShootModule::calculate(const roboteam_msgs::WorldRobot& bot, const World& world) {
	Vector2 botPos(bot.pos);
	Vector2 goalPos = LastWorld::get_our_goal_center();

	auto obstacles = getObstaclesBetweenPoints(botPos, goalPos);
	if (obstacles.size() > 0) {
		return { myConfig().doubles["canShootDanger"], DANGER_CAN_SHOOT };
	}
	return PartialResult();
}

}
}
