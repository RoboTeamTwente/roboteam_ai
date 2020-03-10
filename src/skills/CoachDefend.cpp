//
// Created by rolf on 5-3-19.
//

#include <skills/CoachDefend.h>
#include <coach/defence/DefenceDealer.h>

namespace rtt::ai {
CoachDefend::CoachDefend(std::string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

void CoachDefend::onInitialize() {}

bt::Node::Status CoachDefend::onUpdate() {
    coach::g_DefenceDealer.addDefender(robot->get()->getId());
    auto targetLocation = coach::g_DefenceDealer.getDefenderPosition(robot->get()->getId());
    if (!targetLocation) {
        command.mutable_vel()->set_x(0);
        command.mutable_vel()->set_y(0);
        command.set_w(0);
        publishRobotCommand();
        return bt::Node::Status::Running;
    }

    RobotCommand velocities;
    // choosing numtrees or other
    if (useBasicGtp(targetLocation->first)) {
        velocities = robot->getControllers().getBasicPosController()->getRobotCommand(robot->get()->getId(), targetLocation->first);
    } else {
        velocities = robot->getControllers().getNumTreePosController()->getRobotCommand(robot->get()->getId(), targetLocation->first);
    }
    if ((targetLocation->first - robot->get()->getId()).length() < Constants::ROBOT_RADIUS()) {
        command.mutable_vel()->set_x(0);
        command.mutable_vel()->set_y(0);
        command.set_w(static_cast<float>(control::ControlUtils::constrainAngle(targetLocation->second)));
    } else {
        command.mutable_vel()->set_x(static_cast<float>(velocities.vel.x));
        command.mutable_vel()->set_y(static_cast<float>(velocities.vel.y));

        if ((targetLocation->first - robot->get()->getPos()).length() < Constants::ROBOT_RADIUS()) {
            command.set_w(static_cast<float>(control::ControlUtils::constrainAngle(targetLocation->second)));
        } else {
            command.set_w(velocities.angle);
        }
    }
    publishRobotCommand();
    return bt::Node::Status::Running;
}

bool CoachDefend::useBasicGtp(Vector2 targetLocation) {
    LineSegment driveLine(robot->get()->getPos(), targetLocation);
    if (driveLine.length() >= 0.2) {
        return false;
    }
    // if line intersects our defence area or other robots we do not do it.
    if (FieldComputations::getDefenseArea(*field, true, 0.15, false).doesIntersect(driveLine)) {
        return false;
    }
    auto robots = world_new::World::instance()->getWorld()->getThem();
    for (const auto &worldRobot : robots) {
        if (worldRobot->getId() != robot->get()->getId()) {
            if (driveLine.distanceToLine(worldRobot->getPos()) < Constants::ROBOT_RADIUS()) {
                return false;
            }
        }
    }
    return true;
}
}  // namespace rtt::ai