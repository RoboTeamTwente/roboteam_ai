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
        updateTickTime();
        if (history.size() < HISTORY_SIZE && currentIndex < history.size()) {
            history.emplace_back(std::move(world));
        } else {
            history[currentIndex] = std::move(world);
            currentIndex %= HISTORY_SIZE;
        }
        currentIndex++;
    }

    std::optional<view::WorldDataView> World::getWorld() const noexcept {
        if (currentWorld) {
            return view::WorldDataView{ &*currentWorld };
        } else {
            return std::nullopt;
        }
    }

    WorldData const &World::getHistoryWorld(size_t ticksAgo) const noexcept {
        assert(ticksAgo < 20 && ticksAgo >= 1 && ticksAgo < history.size() && currentIndex > ticksAgo &&
               "Invalid tick");
        // say ticksAgo is 3, then you'd want the currentIndex - 3 index, so
        return history[currentIndex - ticksAgo];
    }

    void World::updateWorld(proto::World &world) {
        std::scoped_lock<std::mutex> lock{ updateMutex };
        WorldData data{ world, *settings, updateMap };
        setWorld(data);
    }

    World::World(Settings *settings)
            : settings{settings}, currentWorld{std::nullopt} {}

    void World::updateFeedback(uint8_t robotId, proto::RobotFeedback &feedback) {
        std::scoped_lock<std::mutex> lock{ updateMutex };
        updateMap[robotId] = std::move(feedback);
    }

    void World::updateTickTime() noexcept {
        std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
        tickDuration = std::chrono::duration_cast<std::chrono::nanoseconds>(now - lastTick);
        lastTick = now;
    }

    const std::chrono::nanoseconds &World::getTickDuration() const noexcept {
        return tickDuration;
    }
} // namespace rtt::world
