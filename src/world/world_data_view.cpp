//
// Created by john on 1/6/20.
//

#include "roboteam_world/world/world_data_view.hpp"
#include "roboteam_world/world/world_data.hpp"

namespace rtt::world::view {

    std::vector<const rtt::world::robot::Robot *> const &WorldDataView::getUs() const noexcept {
        return data->getUs();
    }

    std::vector<const rtt::world::robot::Robot *> const &WorldDataView::getThem() const noexcept {
        return data->getThem();
    }

    std::vector<rtt::world::robot::Robot> const &WorldDataView::getRobots() const noexcept {
        return data->getRobots();
    }

    std::optional<rtt::world::ball::Ball> const &WorldDataView::getBall() const noexcept {
        return data->getBall();
    }

    std::optional<robot::Robot const *> WorldDataView::getRobotForId(uint8_t id, bool ourTeam) const noexcept {
        if (ourTeam) {
            auto robot = std::find(getUs().begin(), getUs().end(), [&](auto const &rbt) {
                return rbt->id == id;
            });
            return robot == getUs().end() ? std::optional<robot::Robot const *>() : *robot;
        } else {
            auto robot = std::find(getThem().begin(), getThem().end(), [&](auto const &rbt) {
                return rbt->id == id;
            });
            return robot == getThem().end() ? std::optional<robot::Robot const *>() : *robot;
        }
    }

    std::vector<const robot::Robot *>
    WorldDataView::getRobotsForIds(std::set<uint8_t> const &_robots, bool ourTeam) const noexcept {
        auto isInList = [&](const robot::Robot *ptr) {
            return std::find(_robots.begin(), _robots.end(), [&](auto robotId) {
                return robotId == ptr->getId();
            }) != _robots.end();
        };

        std::vector<const robot::Robot *> retRobots{};
        if (ourTeam) {
            std::copy_if(getUs().begin(), getUs().end(), retRobots.begin(), isInList);
        } else {
            std::copy_if(getThem().begin(), getThem().end(), retRobots.begin(), isInList);
        }
        return retRobots;
    }
}