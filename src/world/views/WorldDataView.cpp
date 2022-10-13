//
// Created by john on 1/6/20.
//

#include "world/views/WorldDataView.hpp"

#include "stp/constants/ControlConstants.h"
#include "world/WorldData.hpp"

namespace rtt::world::view {

std::vector<view::RobotView> const &WorldDataView::getUs() const noexcept { return data->getUs(); }

std::vector<view::RobotView> const &WorldDataView::getThem() const noexcept { return data->getThem(); }

std::optional<view::BallView> WorldDataView::getBall() const noexcept { return data->getBall(); }

std::optional<view::RobotView> WorldDataView::getRobotForId(uint8_t id, bool ourTeam) const noexcept {
    auto const &_data = ourTeam ? getUs() : getThem();  // const&, prevents copy

    // return if the lambda evaluates to true
    auto _rbt = std::find_if(_data.begin(), _data.end(), [id](RobotView rbt) { return rbt->getId() == id; });

    // not found ? None, else Some(*_robot);
    return _rbt == _data.end() ? std::optional<RobotView>() : *_rbt;
}

WorldData const &WorldDataView::operator*() const noexcept { return *get(); }

WorldData const *WorldDataView::operator->() const noexcept { return get(); }

WorldData const *WorldDataView::get() const noexcept { return data; }

rtt::world::view::WorldDataView::operator bool() const noexcept { return get() != nullptr; }

std::optional<RobotView> WorldDataView::getRobotClosestToPoint(const Vector2 &point, Team team) const noexcept {
    std::vector<RobotView> robots{};
    robots.reserve(ai::stp::control_constants::MAX_ROBOT_COUNT * 2);
    if (team == us)
        robots.assign(getUs().begin(), getUs().end());
    else if (team == them)
        robots.assign(getThem().begin(), getThem().end());
    else
        robots.assign(getRobotsNonOwning().begin(), getRobotsNonOwning().end());

    return getRobotClosestToPoint(point, robots);
}

std::optional<RobotView> WorldDataView::getRobotClosestToBall(Team team) const noexcept { return getRobotClosestToPoint((*getBall())->position, team); }

std::optional<RobotView> WorldDataView::whichRobotHasBall(Team team) const {
    if (!getBall()) return std::nullopt;

    std::vector<RobotView> robots;
    if (team == us) {
        robots = getUs();
    } else if (team == them) {
        robots = getThem();
    } else {
        robots = getRobotsNonOwning();
    }

    double bestDistance = 9e9;
    RobotView bestRobot = RobotView{nullptr};
    for (auto &robot : robots) {
        if (robot->hasBall()) {
            auto distanceToBall = robot->getDistanceToBall();
            if (distanceToBall < bestDistance){
                bestDistance = distanceToBall;
                bestRobot = robot;
            }
        }
    }

    return bestRobot.get() == nullptr ? std::nullopt : std::optional<RobotView>(bestRobot);
}

std::optional<RobotView> WorldDataView::getRobotClosestToPoint(const Vector2 &point, const std::vector<RobotView> &robots) const noexcept {
    if (robots.empty()) return std::nullopt;

    size_t bestIndex = 0;
    double closest = 9e9;
    for (size_t i = 0; i < robots.size(); i++) {
        if (auto currentBot = robots[i]) {
            double distance = (currentBot->getPos() - point).length();
            if (distance < closest) {
                closest = distance;
                bestIndex = i;
            }
        }
    }

    return robots[bestIndex];
}

const std::vector<RobotView> &WorldDataView::getRobotsNonOwning() const noexcept { return data->getRobotsNonOwning(); }

WorldDataView::WorldDataView(WorldData const *_ptr) noexcept : data{_ptr} {}

}  // namespace rtt::world::view