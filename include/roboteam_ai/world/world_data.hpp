//
// Created by john on 12/16/19.
//

#ifndef RTT_WORLD_DATA_HPP
#define RTT_WORLD_DATA_HPP

#include <vector>

#include "roboteam_proto/World.pb.h"
#include "roboteam_proto/Setting.pb.h"
#include "roboteam_proto/RobotFeedback.pb.h"

#include "settings.hpp"
#include "../roboteam_world/include/roboteam_world/world/robot.hpp"
#include "../roboteam_world/include/roboteam_world/world/ball.hpp"



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
    struct WorldData {
    private:
        std::vector<rtt::world::robot::Robot> robots;
        std::vector<const rtt::world::robot::Robot*> us;
        std::vector<const rtt::world::robot::Robot*> them;

        std::optional<rtt::world::ball::Ball> ball;

        uint64_t time;
    public:
        WorldData(proto::World &protoMsg, settings::Settings const& settings, std::unordered_map<uint8_t, proto::RobotFeedback>& feedback) noexcept;
        std::vector<const rtt::world::robot::Robot*> const& getUs() const noexcept;
        std::vector<const rtt::world::robot::Robot*> const& getThem() const noexcept;
        std::vector<rtt::world::robot::Robot> const& getRobots() const noexcept;
        std::optional<ball::Ball> const& getBall() const noexcept;
    };

} // namespace rtt::world


#endif //RTT_WORLD_DATA_HPP
