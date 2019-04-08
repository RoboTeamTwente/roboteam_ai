#include "CanShootToTarget.h"

namespace rtt {
namespace ai {

CanShootToTarget::CanShootToTarget(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { };

bt::Node::Status CanShootToTarget::onUpdate() {

	if (properties->hasString("target")) {
		target = properties->getString("target");
	} else {
		return bt::Node::Status::Failure;
	}

	if (properties->hasString("margin")) {
		margin = properties->getString("margin");
	} else {
		margin = Constants::ROBOT_RADIUS;
	}
	
	bool canShoot = control::ControlUtils::clearLine(robot.pos, target, world::world->get_world(), margin, true);

	return canShoot ? bt::Node::Status::Success : bt::Node::Status::Failure;
}


}//ai
}//rtt