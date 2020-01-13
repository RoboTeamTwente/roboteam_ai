//
// Created by john on 1/6/20.
//

#include <include/roboteam_ai/world/ball.hpp>
#include "world/views/world_data_view.hpp"
#include "world/world_data.hpp"

namespace rtt::world_new::view {

    std::vector<const rtt::world_new::robot::Robot *> const &WorldDataView::getUs() const noexcept {
        return data->getUs();
    }

    std::vector<const rtt::world_new::robot::Robot *> const &WorldDataView::getThem() const noexcept {
        return data->getThem();
    }

    std::vector<rtt::world_new::robot::Robot> const &WorldDataView::getRobots() const noexcept {
        return data->getRobots();
    }

    std::optional<rtt::world_new::ball::Ball> const &WorldDataView::getBall() const noexcept {
        return data->getBall();
    }

    std::optional<robot::Robot const *> WorldDataView::getRobotForId(uint8_t id, bool ourTeam) const noexcept {
        if (ourTeam) {
            auto robot = std::find_if(getUs().begin(), getUs().end(), [&](auto const &rbt) {
                return rbt->getId() == id;
            });
            return robot == getUs().end() ? std::optional<robot::Robot const *>() : *robot;
        } else {
            auto robot = std::find_if(getThem().begin(), getThem().end(), [&](auto const &rbt) {
                return rbt->getId() == id;
            });
            return robot == getThem().end() ? std::optional<robot::Robot const *>() : *robot;
        }
    }

    std::vector<const robot::Robot *>
    WorldDataView::getRobotsForIds(std::set<uint8_t> const &_robots, bool ourTeam) const noexcept {
        auto isInSet = [&](const robot::Robot* ptr) {
            return static_cast<bool>(_robots.count(ptr->getId()));
        };

        std::vector<const robot::Robot *> retRobots{};
        if (ourTeam) {
            std::copy_if(getUs().begin(), getUs().end(), retRobots.begin(), isInSet);
        } else {
            std::copy_if(getThem().begin(), getThem().end(), retRobots.begin(), isInSet);
        }
        return retRobots;
    }

    WorldDataView::WorldDataView(WorldData const * const _ptr) noexcept
            : data{ _ptr } {
        assert(_ptr && "WorldDatat const* _ptr in explicit WorldDataView(WorldDatat const* _ptr) was null");
    }
}