//
// Created by john on 12/16/19.
//

#include "world/world.hpp"
#include "world/views/world_data_view.hpp"
#include "settings/settings.hpp"

#include <utility>

namespace rtt::world_new {
    WorldData const &World::setWorld(WorldData& newWorld) noexcept {
        if (currentWorld) {
            toHistory(currentWorld.value());
        }
        currentWorld = std::move(newWorld);
        return currentWorld.value();
    }

    void World::toHistory(WorldData &world) noexcept {
        std::scoped_lock<std::mutex> lock{ updateMutex };
        updateTickTime();
        if (history.size() < HISTORY_SIZE && currentIndex < history.size()) {
            history.emplace_back(std::move(world));
        } else {
            history[currentIndex] = std::move(world);
            currentIndex %= HISTORY_SIZE;
        }
        currentIndex++;
    }

    std::optional<view::WorldDataView> World::getWorld() noexcept {
        if (currentWorld) {
            return view::WorldDataView{ &*currentWorld };
        } else {
            return std::nullopt;
        }
    }

    view::WorldDataView World::getHistoryWorld(size_t ticksAgo) noexcept {
        if (!(ticksAgo < 20 && ticksAgo >= 1 && ticksAgo < history.size() && currentIndex > ticksAgo)) {
            return view::WorldDataView{ nullptr };
        };
        // say ticksAgo is 3, then you'd want the currentIndex - 3 index, so
        size_t index = currentIndex - ticksAgo;
        if (index < 0) {
            index = (20 - index);
        }
        return view::WorldDataView(&history[index]);
    }

    void World::updateWorld(proto::World &protoWorld) {
        std::scoped_lock<std::mutex> lock{ updateMutex };
        WorldData data{ protoWorld, *settings, updateMap };
        setWorld(data);
    }

    World::World(Settings *settings)
            : settings{settings}, currentWorld{std::nullopt}, lastTick{ 0 } {}

    void World::updateFeedback(uint8_t robotId, proto::RobotFeedback &feedback) {
        std::scoped_lock<std::mutex> lock{ updateMutex };
        updateMap[robotId] = std::move(feedback);
    }

    void World::updateTickTime() noexcept {
        if (!getWorld()) { // no world currently
            return;
        }

        if (lastTick == 0) { // last tick not set yet
            lastTick = (*getWorld())->getTime();
            return;
        }

        tickDuration = (*getWorld())->getTime() - lastTick;
        lastTick = (*getWorld())->getTime();
    }

    uint64_t World::getTickDuration() const noexcept {
        return tickDuration;
    }

    uint64_t World::getTimeDifference() const noexcept {
        return getTickDuration();
    }
} // namespace rtt::world
