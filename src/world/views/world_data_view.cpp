//
// Created by john on 1/6/20.
//

#include <include/roboteam_ai/world/ball.hpp>
#include "world/views/world_data_view.hpp"
#include "world/world_data.hpp"

namespace rtt::world_new::view {

    std::vector<view::RobotView> const &WorldDataView::getUs() const noexcept {
        return data->getUs();
    }

    std::vector<view::RobotView> const &WorldDataView::getThem() const noexcept {
        return data->getThem();
    }

    std::vector<robot::Robot> const &WorldDataView::getRobots() const noexcept {
        return data->getRobots();
    }

    std::optional<view::BallView> WorldDataView::getBall() const noexcept {
        return data->getBall();
    }

    std::optional<view::RobotView> WorldDataView::getRobotForId(uint8_t id, bool ourTeam) const noexcept {
        auto const &_data = ourTeam ? getUs() : getThem(); // const&, prevents copy

        // return if the lambda evaluates to true
        auto _rbt = std::find_if(_data.begin(), _data.end(), [id](RobotView rbt) {
            return rbt->getId() == id;
        });

        // not found ? None, else Some(*_robot);
        return _rbt == _data.end() ? std::optional<RobotView>() : *_rbt;
    }


    std::vector<RobotView>
    WorldDataView::getRobotsForIds(std::set<uint8_t> const &_robots, bool ourTeam) const noexcept {
        auto isInSet = [&](RobotView view) {
            return static_cast<bool>(_robots.count(view->getId()));
        };

        std::vector<RobotView> retRobots{};
        auto const &_data = ourTeam ? getUs() : getThem();
        std::copy_if(_data.begin(), _data.end(), retRobots.begin(), isInSet);
        return retRobots;
    }

    WorldDataView::WorldDataView(WorldData const *const _ptr) noexcept
            : data{_ptr} {
        assert(_ptr && "WorldDatat const* _ptr in explicit WorldDataView(WorldDatat const* _ptr) was null");
    }

    WorldDataView &WorldDataView::operator=(WorldDataView const &o) noexcept {
        if (this == &o) {
            return *this;
        }
        return *this;
    }

    WorldDataView &WorldDataView::operator=(WorldDataView &&o) noexcept {
        if (this == &o) {
            return *this;
        }
        return *this;
    }

    WorldDataView::WorldDataView(WorldDataView &&o) noexcept
            : data{o.data} {}
}