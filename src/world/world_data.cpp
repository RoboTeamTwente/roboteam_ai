//
// Created by john on 12/16/19.
//

#include "world/team.hpp"
#include "world/world_data.hpp"
#include "include/roboteam_ai/settings/settings.hpp"
#include "world/robot.hpp"

namespace rtt::world_new {
    WorldData::WorldData(proto::World &protoMsg, rtt::Settings const &settings,
                         std::unordered_map<uint8_t, proto::RobotFeedback> &feedback) noexcept
            : time{protoMsg.time()} {
        auto genevaState = 3;

        auto &ours = settings.isYellow() ? protoMsg.yellow() : protoMsg.blue();
        auto &others = settings.isYellow() ? protoMsg.blue() : protoMsg.yellow();

        for (auto &each : ours) {
            us.emplace_back(&robots.emplace_back(feedback, each, team::us, genevaState));
        }

        for (auto &each : others) {
            them.emplace_back(&robots.emplace_back(feedback, each, team::them, genevaState));
        }

        if (protoMsg.has_ball()) {
            ball = ball::Ball{protoMsg.ball()};
        } else {
            ball = std::nullopt;
        }
    }

    std::vector<view::RobotView> const &WorldData::getUs() const noexcept {
        return us;
    }

    std::vector<view::RobotView> const &WorldData::getThem() const noexcept {
        return them;
    }

    std::vector<rtt::world_new::robot::Robot> const &WorldData::getRobots() const noexcept {
        return robots;
    }

    std::optional<view::BallView> WorldData::getBall() const noexcept {
        if (ball.has_value()) {
            return std::optional<view::BallView>(&ball.value());
        } else {
            return std::nullopt;
        }
    }

    bool WorldData::weHaveRobots() const noexcept {
        return !getUs().empty();
    }

    uint64_t WorldData::getTime() const noexcept {
        return time;
    }
} // namespace rtt::world