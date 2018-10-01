#include "CanShootModule.h"
#include "roboteam_utils/world_analysis.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Position.h"
#include "roboteam_utils/Section.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Math.h"
#include "roboteam_msgs/GeometryFieldSize.h"

namespace rtt {
namespace ai {
namespace dangerfinder {

REGISTER_MODULE("CanShoot", CanShootModule)

CanShootModule::CanShootModule() : DangerModule("CanShoot") {}

bool facingGoal(rtt::Position pos) {
	auto geom = rtt::LastWorld::get_field();

    rtt::Section goalSection {
    	-geom.field_length / 2,  geom.goal_width / 2,
		-geom.field_length / 2, -geom.goal_width / 2
    };
    rtt::Vector2 longVec { 100, 0 };
    longVec = longVec.rotate(pos.rot);
    longVec = longVec + pos.location();
	rtt::Vector2 isect = goalSection.intersection({pos.x, pos.y, longVec.x, longVec.y});
    return goalSection.pointOnLine(isect);
}

PartialResult CanShootModule::calculate(const roboteam_msgs::WorldRobot& bot, const roboteam_msgs::World& world) {
	rtt::Vector2 botPos(bot.pos);
	rtt::Vector2 goalPos = rtt::LastWorld::get_our_goal_center();

	auto obstacles = getObstaclesBetweenPoints(botPos, goalPos);

	bool canShoot = obstacles.size() == 0 && rtt::bot_has_ball(bot, world.ball) && facingGoal({bot.pos.x, bot.pos.y, bot.angle});

	if (canShoot) {
		return { (double) myConfig().ints["canShootDanger"], DANGER_CAN_SHOOT };
	}
	return PartialResult();
}


} // dangerfinder
} // ai
} // rtt
