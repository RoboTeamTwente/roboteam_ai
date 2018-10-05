#include "OrientationModule.h"
#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_utils/Position.h"

namespace rtt {
namespace ai {
namespace dangerfinder {

OrientationModule::OrientationModule(double factor, double scalar, double danger) : factor(factor), scalar(scalar), DangerModule(danger) {}

PartialResult OrientationModule::calculate(const roboteam_msgs::WorldRobot& bot, const roboteam_msgs::World& world) {
	static auto geom = LastWorld::get_field();
	Vector2 goalCenter(-geom.field_length / 2, 0);
	Vector2 toGoal = (goalCenter - Vector2(bot.pos));
	double targetAngle = toGoal.angle();
	double angleDiff = fabs(targetAngle - bot.angle);
	double x = (angleDiff - M_PI) / factor;
	return { (x*x/scalar), DANGER_NONE};
}

} // dangerfinder
} // ai
} // rtt
