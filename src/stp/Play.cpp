//
// Created by john on 3/9/20.
//

#include "include/roboteam_ai/stp/Play.hpp"

namespace rtt::ai::stp {

void Play::initialize() noexcept {
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

    if(world->getWorld()->getUs().size() != stpInfos.size()) {
        RTT_WARNING("Reassigning bots");
        assignRoles();
    }

    calculateInfoForPlay();

    for (auto& each : roles) {
        auto roleName{each->getName()};
        if(stpInfos.find(roleName) != stpInfos.end()) {
            // TODO refresh robots in a neater way than is done now.
            // This is necessary to prevent the robotview from going out of scope
            stpInfos.find(roleName)->second.setRobot(world->getWorld()->getRobotForId(stpInfos.find(roleName)->second.getRobot()->get()->getId()));
            stpInfos.find(roleName)->second.setField(world->getField());
            stpInfos.find(roleName)->second.setBall(world->getWorld()->getBall());


            auto index = static_cast<size_t>(each->update(stpInfos.find(each->getName())->second));
            count[index] += 1;
        }
    }

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
