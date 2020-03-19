//
// Created by john on 3/9/20.
//

#include "include/roboteam_ai/stp/Play.hpp"

namespace rtt::ai::stp {

void Play::initialize() noexcept {
    calculateInfoForPlay();
    assignRoles();
}

void Play::updateWorld(world_new::World* world) noexcept {
    this->world = world;
    this->field = world->getField().value();
}

Status Play::update() noexcept {
    constexpr static size_t ENUM_COUNT = 4;
    std::array<size_t, ENUM_COUNT> count{};
    std::fill(count.begin(), count.end(), 0);

    if (world->getWorld()->getUs().size() != stpInfos.size()) {
        // Make sure we don't re assign with too many robots
        if(world->getWorld()->getUs().size() > Constants::ROBOT_COUNT()) {
            RTT_ERROR("More robots than ROBOT_COUNT(), aborting update on Play")
            // Make sure the stpInfos is cleared to trigger a reassign whenever
            // the robots don't exceed ROBOT_COUNT anymore
            stpInfos = std::unordered_map<std::string, StpInfo>{};
            return {};
        }
        RTT_WARNING("Reassigning bots")
        assignRoles();
    }

    for (auto& each : roles) {
        auto roleName{each->getName()};
        if(stpInfos.find(roleName) != stpInfos.end()) {
            // TODO refresh robots
            stpInfos[roleName].setRobot(world->getWorld()->getRobotForId(stpInfos.find(roleName)->second.getRobot()->get()->getId()));
            stpInfos[roleName].setBall(world->getWorld()->getBall());
            stpInfos[roleName].setField(world->getField());

            auto index = static_cast<size_t>(each->update(stpInfos.find(each->getName())->second));
            count[index] += 1;
        }
    }
    calculateInfoForPlay();

    if (count[static_cast<size_t>(Status::Success)] == rtt::ai::Constants::ROBOT_COUNT()) {
        return Status::Success;
    }

    if (count[static_cast<size_t>(Status::Failure)]) {
        return Status::Failure;
    }

    if (count[static_cast<size_t>(Status::Waiting)]) {
        return Status::Waiting;
    }

    return Status::Running;
}
}  // namespace rtt::ai::stp
