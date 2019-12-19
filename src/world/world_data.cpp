//
// Created by john on 12/16/19.
//

#include "roboteam_world/world/team.hpp"
#include "roboteam_world/world/world_data.hpp"
#include "roboteam_world/world/settings.hpp"
#include "roboteam_world/world/robot.hpp"

namespace rtt::world {
    WorldData::WorldData(proto::World& protoMsg, settings::Settings const& settings)
        : time{ protoMsg.time() } {
        auto genevaState = 3;

        auto& ours = settings.isYellow() ? protoMsg.yellow() : protoMsg.blue();
        auto& others = settings.isYellow() ? protoMsg.blue() : protoMsg.yellow();

        for (auto& each : ours) {
            us.emplace_back(&robots.emplace_back(each, team::us, genevaState));
        }

        for (auto& each : others) {
            them.emplace_back(&robots.emplace_back(each, team::them, genevaState));
        }

        auto _ball = protoMsg.has_ball();



        // TODO: ball

    }


} // namespace rtt::world