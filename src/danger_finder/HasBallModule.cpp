#include "roboteam_world/danger_finder/HasBallModule.h"
#include "roboteam_utils/world_analysis.h"

namespace rtt {
namespace df {

REGISTER_MODULE("HasBall", HasBallModule);

HasBallModule::HasBallModule() : DangerModule("HasBall") {}

PartialResult HasBallModule::calculate(const roboteam_msgs::WorldRobot& bot, const roboteam_msgs::World& world) {
	// std::cout << "calling bot_has_ball function...";
	bool hasBall = bot_has_ball(bot, world.ball);
	
	// if (hasBall) {
	// 	ROS_INFO_STREAM("robot " << bot.id << " has the ball");
	// }
	return { hasBall ? myConfig().doubles["hasBallDanger"] : 0, hasBall ? DANGER_HAS_BALL : 0 };
}

}
}
