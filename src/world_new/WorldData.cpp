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

        // If there is a ball in the protobuf message, add it to the world
        if (protoMsg.has_ball()) {
            ball = ball::Ball{protoMsg.ball()};
        } else {
            ball = std::nullopt;
        }
        auto amountUs = ours.size();
        auto amountThem = others.size();
        robots.reserve(amountUs + amountThem);

        for (auto &each : ours) {
            robots.emplace_back(feedback, each, Team::us, getBall());
        }
        for (auto &each : others) {
            robots.emplace_back(feedback, each, Team::them, getBall());
        }

us.reserve(amountUs);
them.reserve(amountThem);
        setViewVectors();
    }

    std::vector<view::RobotView> const &WorldData::getUs() const noexcept {
        return us;
    }

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

    std::vector<view::RobotView> const &WorldData::getRobotsNonOwning() const noexcept { return robotsNonOwning; }

    WorldData &WorldData::operator=(WorldData const & other) noexcept {
        if (this == &other) {
            return *this;
        }

        this->robots = other.robots;
        this->ball = other.ball;

        this->us.reserve(other.getUs().size());
        this->them.reserve(other.getThem().size());

        setViewVectors();
        return *this;
    }

    WorldData::WorldData(WorldData const & other) noexcept :
            robots{ other.robots }, ball{ other.ball }{
        us.reserve(other.getUs().size());
        them.reserve(other.getThem().size());
        setViewVectors();
    }

    void WorldData::setViewVectors() noexcept {
        /*
         * Reserve the us and them vectors, making sure they are not being resized (which will invalidate the references)
         */
        for (auto const& each : robots) {
            robotsNonOwning.emplace_back(&each);
            auto& matchingVector = each.getTeam() == Team::us ? us : them;
            matchingVector.emplace_back(&each);
        }
    }
}  // namespace rtt::world_new
