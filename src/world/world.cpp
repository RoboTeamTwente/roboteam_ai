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
        history[currentIndex++] = std::move(world);
        currentIndex %= HISTORY_SIZE;
    }

    WorldData const &World::getWorld() const noexcept {
        return currentWorld;
    }
} // namespace rtt::world
