//
// Created by john on 12/16/19.
//

#include "world_new/World.hpp"

#include <utility>

#include "Settings/Settings.h"
#include "world_new/views/WorldDataView.hpp"

namespace rtt::world_new {
WorldData const &World::setWorld(WorldData &newWorld) noexcept {
//    std::lock_guard mtx{ updateMutex };
    if (currentWorld) {
        toHistory(currentWorld.value());
    }
    currentWorld = std::move(newWorld);
    return currentWorld.value();
}

void World::toHistory(WorldData &world) noexcept {
//    std::lock_guard mtx{ updateMutex };
    updateTickTime();
    if (history.size() < HISTORY_SIZE && currentIndex < history.size()) {
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

view::WorldDataView World::getHistoryWorld(size_t ticksAgo) const noexcept {
    if (ticksAgo > history.size()) {
        return view::WorldDataView(nullptr);
    }
    // say ticksAgo is 3, then you'd want the currentIndex - 3 index, so
    return view::WorldDataView(&history[currentIndex - ticksAgo]);
}

void World::updateWorld(proto::World &protoWorld) {
    WorldData data{protoWorld, *settings, updateMap};
    setWorld(data);
}

World::World(Settings *settings) : settings{settings}, currentWorld{std::nullopt}, lastTick{0} {}

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

robot::RobotControllers &World::getControllersForRobot(uint8_t id) noexcept {
    return robotControllers[id];
}
}  // namespace rtt::world_new
