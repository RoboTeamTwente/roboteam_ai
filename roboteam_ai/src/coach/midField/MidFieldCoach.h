//
// Created by robzelluf on 5/27/19.
//

#ifndef ROBOTEAM_AI_MIDFIELDCOACH_H
#define ROBOTEAM_AI_MIDFIELDCOACH_H

#include <roboteam_utils/Vector2.h>
#include <roboteam_ai/src/world/Field.h>
#include <roboteam_ai/src/control/ControlUtils.h>

namespace rtt {
namespace ai {
namespace coach {

class MidFieldCoach {
    private:
        const double DISTANCE_FROM_MIDDLE_LINE = 3.0;
        const double HARASSER_SECONDS_AHEAD = 0.5;
        const double STAND_STILL_DISTANCE = Constants::ROBOT_RADIUS();
        const double MIN_OPPONENT_VELOCITY = 0.5;
        const double DEFAULT_HARASS_DISTANCE = 4 * Constants::ROBOT_RADIUS();

        using Robot = world::Robot;
        using RobotPtr = std::shared_ptr<Robot>;
        using Ball = world::Ball;
        using BallPtr = std::shared_ptr<Ball>;

        struct HarassTarget {
            Vector2 harassPosition;
            int harassRobot;
        };

        enum HarassType {
            HARASS,
            BLOCK_PASS,
            BALL
        };

        std::vector<RobotPtr> currentMidfielders;
        std::map<int, Vector2> targetPositions;
        std::map<int, RobotPtr> targetRobotsToHarass;
    public:
        void addMidFielder(RobotPtr &thisRobot);
        void removeMidFielder(RobotPtr &thisRobot);
        HarassTarget getTargetPosition(RobotPtr &thisRobot);
        bool validOpponent(const RobotPtr& opponent);
        HarassTarget harassRobot(const RobotPtr& thisRobot, const RobotPtr& opponent);
        RobotPtr findRobotToHarass(const RobotPtr& thisRobot);
        Vector2 standFree(const RobotPtr& thisRobot);
        HarassType getHarassType(const RobotPtr& thisRobot, const RobotPtr& opponent);
};

extern MidFieldCoach g_midFieldCoach;

}
}
}


#endif //ROBOTEAM_AI_MIDFIELDCOACH_H
