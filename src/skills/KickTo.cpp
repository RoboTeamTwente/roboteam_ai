#include <skills/KickTo.h>
#include <world_new/FieldComputations.hpp>

namespace rtt::ai {

KickTo::KickTo(std::string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}
void KickTo::onInitialize() {
    std::string type = properties->getString("type");
    if (type == "shootout") {
        shootPos = Vector2((*field).getFieldLength() * 0.2, 0);  // 2.4 m for A field, 1.8 for B
    } else {
        shootPos = Vector2(0, 0);
    }
}
/// Get an update on the skill
bt::Node::Status KickTo::onUpdate() {
    if (world_new::FieldComputations::pointIsInDefenceArea(*field, ball->get()->getPos(), false)) {
        command.set_w(0);
        publishRobotCommand();
        return Status::Running;
    }

    Vector2 aimPoint = shootPos;
    // TODO: tune kick velocity
    auto shotData =
        robot->getControllers().getShotController()->getRobotCommand(robot->get()->getId(), aimPoint, false, control::BallSpeed::BALL_PLACEMENT, true, control::ShotPrecision::HIGH);
    command = shotData.makeROSCommand();
    publishRobotCommand();
    return Status::Running;
}

}  // namespace rtt::ai