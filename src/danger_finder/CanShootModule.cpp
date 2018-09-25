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

// Returns true if a robot is facing the goal.
bool facingGoal(Position position) {
	// get the last world_state
	auto lastWorldField = LastWorld::get_field();

    Section goalSection {
    	-lastWorldField.field_length / 2,  lastWorldField.goal_width / 2,
		-lastWorldField.field_length / 2, -lastWorldField.goal_width / 2
    };
    Vector2 longVec { 100, 0 };
    longVec = longVec.rotate(position.rot);
    longVec = longVec + position.location();
    // Computes the intersection point between extension of rotation robot and goal
    Vector2 intersection = goalSection.intersection({position.x, position.y, longVec.x, longVec.y});
    return goalSection.pointOnLine(intersection);
}

PartialResult CanShootModule::calculate(const roboteam_msgs::WorldRobot& bot, const roboteam_msgs::World& world) {
	// find all obstacles between robot and goal.
	Vector2 botPos(bot.pos);
	Vector2 goalPos = LastWorld::get_our_goal_center();
	auto obstacles = getObstaclesBetweenPoints(botPos, goalPos);
	// A robot can shoot if it has the ball and is facing the goal and there are no obstacles in between
	bool canShoot = obstacles.size() == 0 && bot_has_ball(bot, world.ball) && facingGoal({bot.pos.x, bot.pos.y, bot.angle});
	// Sets flag and edits partialresult if neccessary
	if (canShoot) {
		return { (double) myConfig().ints["canShootDanger"], DANGER_CAN_SHOOT };
	}
	return PartialResult();
}

}
}
