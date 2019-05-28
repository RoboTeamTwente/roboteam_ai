//
// Created by robzelluf on 5/27/19.
//

#ifndef ROBOTEAM_AI_MIDFIELDCOACH_H
#define ROBOTEAM_AI_MIDFIELDCOACH_H

#include <roboteam_utils/Vector2.h>
#include <roboteam_ai/src/world/Field.h>
#include <roboteam_ai/src/control/ControlUtils.h>
#include <roboteam_ai/src/world/BallPossession.h>

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

        struct Target {
            Vector2 targetPosition;
            int targetRobot;
        };

        enum HarassType {
            HARASS_OFFENSIVE,
            HARASS_DEFENSIVE,
            BLOCK_PASS,
            BALL,
            STAND_FREE
        };

        std::vector<RobotPtr> currentMidfielders;
        std::map<int, Vector2> targetPositions;
        std::map<int, RobotPtr> targetRobotsToHarass;

        Target getBall(RobotPtr &thisRobot, const RobotPtr& opponent);
        Target standFree(const RobotPtr &thisRobot);
        MidFieldCoach::Target
        harassRobot(const RobotPtr &thisRobot, const RobotPtr &opponent, HarassType harassType) const;
        Target blockPass(const RobotPtr &thisRobot, const RobotPtr &opponent, const BallPtr &ball) const;
    public:
        void addMidFielder(RobotPtr &thisRobot);
        void removeMidFielder(RobotPtr &thisRobot);
        bool validOpponent(const RobotPtr& opponent);

        RobotPtr findRobotToHarass(const RobotPtr& thisRobot);
        HarassType getHarassType(const RobotPtr& thisRobot, const RobotPtr& opponent);


    Target getTargetPosition(RobotPtr &thisRobot);
};

extern MidFieldCoach g_midFieldCoach;

}
}
}


#endif //ROBOTEAM_AI_MIDFIELDCOACH_H
