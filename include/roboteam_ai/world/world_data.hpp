//
// Created by john on 12/16/19.
//

#ifndef RTT_WORLD_DATA_HPP
#define RTT_WORLD_DATA_HPP

#include <vector>

#include "roboteam_proto/World.pb.h"
#include "roboteam_proto/Setting.pb.h"
#include "roboteam_proto/RobotFeedback.pb.h"

#include "world/robot.hpp"
#include "world/ball.hpp"

namespace rtt::world_new {

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
    private:
        std::vector<rtt::world_new::robot::Robot> robots;
        std::vector<const rtt::world_new::robot::Robot*> us;
        std::vector<const rtt::world_new::robot::Robot*> them;

        std::optional<rtt::world_new::ball::Ball> ball;

        uint64_t time{};
    public:
        WorldData() = default;

        WorldData& operator=(WorldData const&) = delete;
        WorldData(WorldData const&) = delete;

        WorldData(WorldData&& old) noexcept;
        WorldData& operator=(WorldData&&) = default;

        WorldData(proto::World &protoMsg, rtt::Settings const& settings, std::unordered_map<uint8_t, proto::RobotFeedback>& feedback) noexcept;
        [[nodiscard]] std::vector<const rtt::world_new::robot::Robot*> const& getUs() const noexcept;
        [[nodiscard]] std::vector<const rtt::world_new::robot::Robot*> const& getThem() const noexcept;
        [[nodiscard]] std::vector<rtt::world_new::robot::Robot> const& getRobots() const noexcept;
        [[nodiscard]] std::optional<ball::Ball> const& getBall() const noexcept;
    };

} // namespace rtt::world


#endif //RTT_WORLD_DATA_HPP
