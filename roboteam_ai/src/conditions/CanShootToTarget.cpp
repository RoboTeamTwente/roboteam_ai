#include "CanShootToTarget.h"

namespace rtt {
namespace ai {

CanShootToTarget::CanShootToTarget(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { };

bt::Node::Status CanShootToTarget::onUpdate() {

    if (properties->hasVector2("target")) {
		target = properties->getVector2("target");
	} else {
		return bt::Node::Status::Failure;
	}

	if (properties->hasDouble("margin")) {
		margin = properties->getDouble("margin");
	} else {
		margin = MAX_DIST_FROM_LINE;
	}

	bool canShoot = control::ControlUtils::clearLine(robot->pos, target, world::world->getWorld(), margin, true);
	return canShoot ? bt::Node::Status::Success : bt::Node::Status::Failure;
}


}//ai
}//rtt