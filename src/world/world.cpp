//
// Created by john on 12/16/19.
//

#include "roboteam_world/world/world.hpp"

namespace rtt::world {
    WorldData const& World::setWorld(WorldData &newWorld) noexcept {
        toHistory(currentWorld);
        currentWorld = std::move(newWorld);
        return currentWorld;
    }

    void World::toHistory(WorldData &world) noexcept {
        if (history.size() < HISTORY_SIZE && currentIndex < history.size()) {
            history.emplace_back(std::move(world));
        } else {
            history[currentIndex] = std::move(world);
            currentIndex %= HISTORY_SIZE;
        }
        currentIndex++;
    }

    WorldData const &World::getWorld() const noexcept {
        return currentWorld;
    }

    WorldData const &World::getHistoryWorld(size_t ticksAgo) const noexcept {
        assert(ticksAgo < 20 && ticksAgo >= 1 && ticksAgo < history.size() && currentIndex > ticksAgo && "Invalid tick");
        // say ticksAgo is 3, then you'd want the currentIndex - 3 index, so
        return history[currentIndex - ticksAgo];
    }
} // namespace rtt::world
