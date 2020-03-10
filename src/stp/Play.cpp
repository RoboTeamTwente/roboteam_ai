//
// Created by john on 3/9/20.
//

#include "include/roboteam_ai/stp/Play.hpp"

namespace rtt::ai::stp {

void Play::initialize() noexcept {
    for (int i = 0; i < 12; i++) {
        tacticInfos.emplace_back();
    }
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

    // Update for every role
    for (int i = 0; i < roles.size(); i++) {
        // TODO Refreshing of the robotviews is kind of ugly now, right?
        tacticInfos[i].setRobot(world->getWorld()->getRobotForId(tacticInfos[i].getRobot()->get()->getId()));

        // Update the role with the tacticInfo for that role (that was assigned by robotDealer)
        auto index = static_cast<size_t>(roles[i]->update(tacticInfos[i]));
        count[index] += 1;
    }

    if (count[static_cast<size_t>(Status::Success)] == ROBOT_COUNT) {
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
