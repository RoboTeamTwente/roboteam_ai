//
// Created by john on 12/16/19.
//

#include "roboteam_world/world/team.hpp"
#include "roboteam_world/world/world_data.hpp"
#include "roboteam_world/world/settings.hpp"
#include "roboteam_world/world/robot.hpp"

namespace rtt::world {
    WorldData::WorldData(proto::World& protoMsg, settings::Settings const& settings, std::unordered_map<uint8_t, proto::RobotFeedback>& feedback) noexcept
        : time{ protoMsg.time() } {
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

    std::vector<const rtt::world::robot::Robot *> const &WorldData::getUs() const noexcept {
        return us;
    }

    std::vector<const rtt::world::robot::Robot *> const &WorldData::getThem() const noexcept {
        return them;
    }

    std::vector<rtt::world::robot::Robot> const &WorldData::getRobots() const noexcept {
        return robots;
    }

    std::optional<ball::Ball> const &WorldData::getBall() const noexcept {
        return ball;
    }


} // namespace rtt::world