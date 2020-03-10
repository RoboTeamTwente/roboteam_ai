//
// Created by baris on 24-4-19.
//

#include <control/PositionUtils.h>
#include <skills/formations/DefendFreeKick.h>
#include <roboteam_utils/Print.h>

namespace rtt::ai {

std::vector<Vector2> DefendFreeKick::posses;
std::vector<world_new::view::RobotView> rtt::ai::DefendFreeKick::robotsInFormation{};

Vector2 DefendFreeKick::getFormationPosition() {
    auto ballOpt = world_new::World::instance()->getWorld()->getBall();
    if (ballOpt) {
        robot->getControllers().getNumTreePosController()->setAvoidBallDistance(0.55);
        update = true;
        posses = rtt::ai::control::PositionUtils::getDefendFreeKick(*field, ballOpt.value(), robotsInFormation.size());
        return getOptimalPosition(robot->get()->getId(), robotsInFormation, posses);
    }
    RTT_ERROR("No ball found, so freekickformation is not behaving as desired")
    return {};
}

std::vector<world_new::view::RobotView> DefendFreeKick::robotsInFormationPtr() { return robotsInFormation; }

DefendFreeKick::DefendFreeKick(std::string name, bt::Blackboard::Ptr blackboard) : Formation(std::move(name), std::move(blackboard)) {
    robotsInFormation = std::vector<world_new::view::RobotView>();
}

void DefendFreeKick::onTerminate(Skill::Status s) {
    Formation::onTerminate(s);
    update = false;
}
}  // namespace rtt::ai
