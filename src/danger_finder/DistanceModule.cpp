#include "roboteam_world/danger_finder/DistanceModule.h"
#include "roboteam_msgs/GeometryFieldSize.h"

namespace rtt {
namespace df {

DistanceModule::DistanceModule(double factor) : DangerModule("Distance"), factor(factor) {}

inline Vector2 getGoalCenter() {
	static auto geom = LastWorld::get_field();
	return Vector2(geom.field_length / 2, 0);
}

PartialResult DistanceModule::calculate(const roboteam_msgs::WorldRobot& bot, const roboteam_msgs::World& world) {
	double dist = Vector2(bot.pos).dist(getGoalCenter());
	double last = lastDistances[bot.id];
	DangerFlag flag;
	lastDistances[bot.id] = dist;
	if (last != 0.0 && last < dist) {
		flag =  DANGER_CLOSING;
	}
	flag =  DANGER_NONE;
	return {(dist - myConfig().doubles["maxWidth"]) / factor, flag};
}


}
}
