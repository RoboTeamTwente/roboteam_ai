//
// Created by john on 12/16/19.
//

#include "roboteam_world/world/team.hpp"
#include "roboteam_world/world/world_data.hpp"
#include "roboteam_world/world/settings.hpp"
#include "roboteam_world/world/robot.hpp"

namespace rtt::world {
    WorldData::WorldData(proto::World& protoMsg, settings::Settings const& settings, std::unordered_map<uint8_t, proto::RobotFeedback> const& feedback) noexcept
        : time{ protoMsg.time() } {
        auto genevaState = 3;

        auto &ours = settings.isYellow() ? protoMsg.yellow() : protoMsg.blue();
        auto &others = settings.isYellow() ? protoMsg.blue() : protoMsg.yellow();

        for (auto &each : ours) {
            us.emplace_back(&robots.emplace_back(each, team::us, genevaState, feedback));
        }

        for (auto &each : others) {
            them.emplace_back(&robots.emplace_back(each, team::them, genevaState, ));
        }

        if (protoMsg.has_ball()) {
            ball = ball::Ball{protoMsg.ball()};
        } else {
            ball = std::nullopt;
        }
    }

    std::vector<rtt::world::robot::Robot *> const &WorldData::getUs() const noexcept {
        return us;
    }

    std::vector<rtt::world::robot::Robot *> const &WorldData::getThem() const noexcept {
        return them;
    }

    std::vector<rtt::world::robot::Robot> const &WorldData::getRobots() const noexcept {
        return robots;
    }

    std::optional<ball::Ball> const &WorldData::getBall() const noexcept {
        return ball;
    }

    std::optional<robot::Robot const *> WorldData::getRobotForId(uint8_t id, bool ourTeam) const noexcept {
        if (ourTeam) {
            auto robot = std::find(us.begin(), us.end(), [&](auto const &rbt) {
                return rbt->id == id;
            });
            return robot == getUs().end() ? std::optional<robot::Robot *>() : *robot;
        } else {
            auto robot = std::find(getThem().begin(), getThem().end(), [&](auto const &rbt) {
                return rbt->id == id;
            });
            return robot == getThem().end() ? std::optional<robot::Robot *>() : *robot;
        }
    }

    std::vector<const robot::Robot *>
    WorldData::getRobotsForIds(std::initializer_list<uint8_t> const &_robots, bool ourTeam) const noexcept {
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


} // namespace rtt::world