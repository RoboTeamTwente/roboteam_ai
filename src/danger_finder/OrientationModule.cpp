#include "roboteam_world/danger_finder/OrientationModule.h"
#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_utils/Position.h"

namespace rtt {
namespace df {

REGISTER_MODULE("Orientation", OrientationModule)

OrientationModule::OrientationModule(double factor) : DangerModule("Orientation"), factor(factor) {}

PartialResult OrientationModule::calculate(const roboteam_msgs::WorldRobot& bot, const roboteam_msgs::World& world) {
	static auto geom = LastWorld::get_field();
	Vector2 goalCenter(geom.field_length / 2, 0);
	Vector2 toGoal = (goalCenter - Vector2(bot.pos));
	double targetAngle = toGoal.angle();
	double angleDiff = fabs(targetAngle - bot.angle);
	double x = (angleDiff - M_PI) / factor;
	return {x*x / myConfig().doubles["scalar"], DANGER_NONE};
}

}
}
