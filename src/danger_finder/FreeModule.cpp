#include "roboteam_world/danger_finder/FreeModule.h"
#include "roboteam_utils/world_analysis.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {
namespace df {

REGISTER_MODULE("Free", FreeModule)

FreeModule::FreeModule() : DangerModule("Free") {}

PartialResult FreeModule::calculate(const roboteam_msgs::WorldRobot& bot, const roboteam_msgs::World& world) {
	Vector2 ballPos(world.ball.pos);
	Vector2 botPos(bot.pos);
	const auto obstacles = getObstaclesBetweenPoints(botPos, ballPos);
	if (obstacles.size() == 0) {
		return { myConfig().doubles["standingFreeDanger"], DANGER_FREE };
	}
	return PartialResult();
}

}
}
