//
// Created by john on 12/16/19.
//

#include "world_new/World.hpp"

#include <utility>
#include <include/roboteam_ai/world/Field.h>

#include "include/roboteam_ai/utilities/Settings.h"
#include "world_new/views/WorldDataView.hpp"

namespace rtt::world_new {
WorldData const &World::setWorld(WorldData &newWorld) noexcept {
    std::lock_guard mtx{updateMutex};
    if (currentWorld) {
        toHistory(currentWorld.value());
    }
    currentWorld = std::move(newWorld);
    return currentWorld.value();
}

void World::toHistory(WorldData &world) noexcept {
    updateTickTime();
    if (history.size() < HISTORY_SIZE) {
        history.emplace_back(std::move(world));
    } else {
        history[currentIndex] = std::move(world);
    }
    currentIndex++;
    currentIndex %= HISTORY_SIZE;
}

std::optional<view::WorldDataView> World::getWorld() const noexcept {
    if (currentWorld) {
        /**
         * *currentWorld == a ref to the world data
         * &*currentWorld == a pointer to the world data
         */
        return view::WorldDataView{&*currentWorld};
    } else {
        return std::nullopt;
    }
}

std::optional<ai::world::Field> World::getField() const noexcept {
    if (currentField) {
        return currentField;
    } else {
        return std::nullopt;
    }
}

std::optional<view::WorldDataView> World::getHistoryWorld(size_t ticksAgo) const noexcept {
    std::optional<view::WorldDataView> world = std::nullopt;

    if (ticksAgo == 0) {
        world = getWorld();
    } else if (1 <= ticksAgo and ticksAgo <= history.size()) {
        auto index = currentIndex - ticksAgo;
        if (history.size() < index) index += HISTORY_SIZE;  // Wrap-around. 0 wraps around to 18446744073709551598
        world = view::WorldDataView(&history[index]);
    }

    return world;
}

void World::updateWorld(proto::World &protoWorld) {
    WorldData data{protoWorld, *settings, updateMap};
    setWorld(data);

    std::vector<Vector2> robotPositions(getWorld()->getRobotsNonOwning().size());
    std::transform(getWorld()->getRobotsNonOwning().begin(), getWorld()->getRobotsNonOwning().end(), robotPositions.begin(), [](const auto& robot) -> Vector2 { return (robot->getPos()); });
    positionControl.setRobotPositions(robotPositions);
}

void World::updateField(proto::SSL_GeometryFieldSize &protoField) {
    ai::world::Field field(protoField);
    this->currentField = field;
}

World::World(Settings *settings) : settings{settings}, currentWorld{std::nullopt}, lastTick{0} { history.reserve(HISTORY_SIZE); }

void World::updateFeedback(uint8_t robotId, proto::RobotFeedback &feedback) {
    std::scoped_lock<std::mutex> lock{updateMutex};
    updateMap[robotId] = std::move(feedback);
}

void World::updateTickTime() noexcept {
    if (!getWorld()) {  // no world currently
        return;
    }

    if (lastTick == 0) {  // last tick not set yet
        lastTick = (*getWorld())->getTime();
        return;
    }

    tickDuration = (*getWorld())->getTime() - lastTick;
    lastTick = (*getWorld())->getTime();
}

uint64_t World::getTimeDifference() const noexcept { return tickDuration; }

robot::RobotControllers &World::getControllersForRobot(uint8_t id) noexcept { return robotControllers[id]; }

ai::control::PositionControl *World::getRobotPositionController() noexcept {
    return &positionControl;
}

size_t World::getHistorySize() const noexcept { return history.size(); }
}  // namespace rtt::world_new
