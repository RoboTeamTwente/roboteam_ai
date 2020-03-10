#include <control/PositionUtils.h>
#include <skills/formations/FreeKickFormation.h>
#include <roboteam_utils/Print.h>

namespace rtt::ai {

std::vector<Vector2> FreeKickFormation::posses;
std::vector<world_new::view::RobotView> rtt::ai::FreeKickFormation::robotsInFormation{};

Vector2 FreeKickFormation::getFormationPosition() {
    auto ballOpt = world_new::World::instance()->getWorld()->getBall();
    if (ballOpt) {
        update = true;
        posses = rtt::ai::control::PositionUtils::getFreeKickPositions(*field, ballOpt.value(), robotsInFormation.size());
        return getOptimalPosition(robot->get()->getId(), robotsInFormation, posses);
    }
    RTT_ERROR("No ball found, so freekickformation is not behaving as desired")
    return {};
}

std::vector<world_new::view::RobotView> FreeKickFormation::robotsInFormationPtr() { return robotsInFormation; }

FreeKickFormation::FreeKickFormation(std::string name, bt::Blackboard::Ptr blackboard) : Formation(name, blackboard) {
    robotsInFormation = std::vector<world_new::view::RobotView>();
};

void FreeKickFormation::onTerminate(Skill::Status s) {
    Formation::onTerminate(s);
    update = false;
}
}  // namespace rtt::ai
