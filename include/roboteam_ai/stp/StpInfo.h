//
// Created by john on 3/2/20.
//

#ifndef RTT_STPINFO_H
#define RTT_STPINFO_H

#include <roboteam_utils/Rectangle.h>
#include <roboteam_utils/Circle.h>

namespace rtt::ai::stp {
    /**
     * Class used for the Areas to avoid in TacticInfo
     * @tparam T Type of the shape for the areas to avoid
     */
    template<typename T>
    struct Areas {
        std::vector<T> areasToAvoid;

        /**
         * Checks whether the Robot is or would be in any of the shapes
         * @param pos Center of the robot
         * @return any(robot in shape for shape in areas)
         */
        [[nodiscard]] bool isInAny(Vector2 const &pos) const noexcept {
            Circle circle{ pos, Constants::ROBOT_RADIUS() };
            return std::any_of(areasToAvoid.begin(), areasToAvoid.end(), [&](auto const &area) {
                return area.intersects(circle);
            });
        }
    };

    /**
     * Structure that represents the info passed to tactics
     */
    struct TacticInfo {
        /**
         * View to the ball in the world this tactic executes on
         */
        world_new::view::BallView ball;

        /**
         * Robot to pass to
         */
        world_new::view::RobotView passRobot;

        /**
         * Current world data that's used for calculations
         */
        world_new::view::WorldDataView world;

        /**
         * Position, could be position of robot, or whatever position the skill needs, maybe target position
         */
        Vector2 position;

        /**
         * Maximum speed that something can have, the ball, the robot... whatever the skill needs
         */
        float maxSpeed;

        /**
         * Areas to avoid, read Areas<T> for more info
         */
        Areas<Rectangle> areasToAvoid;
    };


    /**
     * Skill info used in skills
     */
    struct SkillInfo {
        /**
         * TacticInfo of the tactic that called the skill
         */
        TacticInfo tactic;
    };
};

#endif // RTT_STPINFO_H
