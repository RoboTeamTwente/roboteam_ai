//
// Created by john on 12/16/19.
//

#ifndef RTT_WORLD_DATA_HPP
#define RTT_WORLD_DATA_HPP

#include <vector>

#include "roboteam_proto/World.pb.h"
#include "roboteam_proto/Setting.pb.h"

#include "settings.hpp"
#include "ball.hpp"


namespace rtt::world {

    namespace robot {
        class Robot;
    } // namespace robot

    /**
     * WorldData structs
     * Hold data about a specific time point in the world.
     * Holds:
     *  Robots
     *  Ball
     *  Timepoint
     */
    class WorldData {
        std::vector<rtt::world::robot::Robot> robots;
        std::vector<rtt::world::robot::Robot*> us;
        std::vector<rtt::world::robot::Robot*> them;

        std::optional<rtt::world::ball::Ball> ball;

        uint64_t time;
    public:
        WorldData(proto::World &protoMsg, settings::Settings const& settings) noexcept;

        std::vector<rtt::world::robot::Robot*> const& getUs() const noexcept;
        std::vector<rtt::world::robot::Robot*> const& getThem() const noexcept;
        std::vector<rtt::world::robot::Robot> const& getRobots() const noexcept;
        std::optional<rtt::world::ball::Ball> const& getBall() const noexcept;

        std::optional<Robot const*> getRobotForId(uint8_t id, bool ourTeam = true) const noexcept;
        std::vector<Robot const*> getRobotsForIds(std::initializer_list<uint8_t> const& robots, bool ourTeam = true) const noexcept;
    };

} // namespace rtt::world


#endif //RTT_WORLD_DATA_HPP
