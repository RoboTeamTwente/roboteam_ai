//
// Created by john on 12/16/19.
//

#include "world/World.hpp"

namespace rtt::world {
WorldData const &World::setWorld(WorldData &newWorld) noexcept {
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
    /**
     * Possibly there's an issue in GCC that std::optional<T>::operator bool causes undefined behavior.
     */
    if (currentWorld.has_value()) {
        /**
         * *currentWorld == a ref to the world data
         * &*currentWorld == a pointer to the world data
         */
        return view::WorldDataView{&*currentWorld};
    } else {
        return std::nullopt;
    }
}

std::optional<world::Field> World::getField() const noexcept {
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
    WorldData data{this, protoWorld, *settings};
    setWorld(data);
}

void World::updateField(proto::SSL_GeometryFieldSize &protoField) {
    world::Field field(protoField);
    this->currentField = field;
}

void World::updateField(rtt::world::Field &protoField) { this->currentField = protoField; }

World::World(Settings *settings) : settings{settings}, currentWorld{std::nullopt}, lastTick{0} { history.reserve(HISTORY_SIZE); }


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

void World::updatePositionControl() {
    std::vector<Vector2> robotPositions(getWorld()->getRobotsNonOwning().size());
    std::transform(getWorld()->getRobotsNonOwning().begin(), getWorld()->getRobotsNonOwning().end(), robotPositions.begin(),
                   [](const auto &robot) -> Vector2 { return (robot->getPos()); });
    positionControl.setRobotPositions(robotPositions);
}

ai::control::PositionControl *World::getRobotPositionController() noexcept { return &positionControl; }

size_t World::getHistorySize() const noexcept { return history.size(); }

}  // namespace rtt::world
