#include "roboteam_world/danger_finder/CanShootModule.h"
#include "roboteam_utils/world_analysis.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Position.h"
#include "roboteam_utils/Section.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Math.h"
#include "roboteam_msgs/GeometryFieldSize.h"

namespace rtt {
namespace df {

REGISTER_MODULE("CanShoot", CanShootModule)

CanShootModule::CanShootModule() : DangerModule("CanShoot") {}

bool facingGoal(Position pos) {
	auto geom = LastWorld::get_field();
    Section goalSection {
    	-geom.field_length / 2,  geom.goal_width / 2,
		-geom.field_length / 2, -geom.goal_width / 2
    };
    Vector2 longVec { 100, 0 };
    longVec = longVec.rotate(pos.rot);
    longVec = longVec + pos.location();
    Vector2 isect = goalSection.intersection({pos.x, pos.y, longVec.x, longVec.y});
    return goalSection.pointOnLine(isect);
}

PartialResult CanShootModule::calculate(const roboteam_msgs::WorldRobot& bot, const roboteam_msgs::World& world) {
	Vector2 botPos(bot.pos);
	Vector2 goalPos = LastWorld::get_our_goal_center();

	auto obstacles = getObstaclesBetweenPoints(botPos, goalPos);
	if (obstacles.size() == 0 && bot_has_ball(bot, world.ball) && facingGoal({bot.pos.x, bot.pos.y, bot.angle})) {
		return { myConfig().ints["canShootDanger"], DANGER_CAN_SHOOT };
	}
	return PartialResult();
}

}
}
