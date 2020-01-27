#include <control/PositionUtils.h>
#include "skills/KickTo.h"
#include <world/Field.h>
#include <control/numTrees/NumTreePosControl.h>
#include <control/BasicPosControl.h>
#include <control/ControlUtils.h>

namespace rtt {
namespace ai {

KickTo::KickTo(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {
}
void KickTo::onInitialize() {
    std::string type=properties->getString("type");
    shootPos = properties->getVector2("where");
}
/// Get an update on the skill
bt::Node::Status KickTo::onUpdate() {
    std::cout << "UPDATE" << std::endl;
    if (field->pointIsInDefenceArea(ball->getPos(), false)) {
        command.set_w(0);
        publishRobotCommand();
        return Status::Running;
    }

    Vector2 aimPoint = shootPos;
    auto shotData = robot->getShotController()->getRobotCommand(*robot, aimPoint, false, control::PASS,
                                                               false, control::HIGH);
    command = shotData.makeROSCommand();
    publishRobotCommand();
    return Status::Running;
}

} // ai
} // rtt