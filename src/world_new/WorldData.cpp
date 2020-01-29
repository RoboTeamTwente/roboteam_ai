//
// Created by john on 12/16/19.
//

#include "world_new/WorldData.hpp"

#include "include/roboteam_ai/utilities/Settings.h"
#include "world_new/Robot.hpp"
#include "world_new/Team.hpp"

namespace rtt::world_new {
WorldData::WorldData(proto::World &protoMsg, rtt::Settings const &settings, std::unordered_map<uint8_t, proto::RobotFeedback> &feedback) noexcept : time{protoMsg.time()} {
    auto &ours = settings.isYellow() ? protoMsg.yellow() : protoMsg.blue();
    auto &others = settings.isYellow() ? protoMsg.blue() : protoMsg.yellow();

    for (auto &each : ours) {
        auto robot = new robot::Robot(feedback, each, Team::us);
        robots.push_back(std::move(*robot));
        us.emplace_back(robot);
    }

    for (auto &each : others) {
        auto robot = new robot::Robot(feedback, each, Team::them);
        robots.push_back(std::move(*robot));
        them.emplace_back(robot);
    }


    if (protoMsg.has_ball()) {
        ball = ball::Ball{protoMsg.ball()};
    } else {
        ball = std::nullopt;
    }
}

std::vector<view::RobotView> const &WorldData::getUs() const noexcept { return us; }

std::vector<view::RobotView> const &WorldData::getThem() const noexcept { return them; }

std::vector<rtt::world_new::robot::Robot> const &WorldData::getRobots() const noexcept { return robots; }

std::optional<view::BallView> WorldData::getBall() const noexcept {
    if (ball.has_value()) {
        return std::optional<view::BallView>(&ball.value());
    } else {
        return std::nullopt;
    }
}

bool WorldData::weHaveRobots() const noexcept { return !getUs().empty(); }

uint64_t WorldData::getTime() const noexcept { return time; }
}  // namespace rtt::world_new
