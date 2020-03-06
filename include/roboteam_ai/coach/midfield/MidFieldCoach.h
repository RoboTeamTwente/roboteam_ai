//
// Created by robzelluf on 5/27/19.
//

#ifndef ROBOTEAM_AI_MIDFIELDCOACH_H
#define ROBOTEAM_AI_MIDFIELDCOACH_H

#include <control/ControlUtils.h>
#include <roboteam_utils/Vector2.h>
#include <world/BallPossession.h>
#include <world/FieldComputations.h>

#include "coach/heuristics/CoachHeuristics.h"

namespace rtt::ai::coach {

class MidFieldCoach {
   private:
    const double DISTANCE_FROM_MIDDLE_LINE = 2.0;
    const double HARASSER_SECONDS_AHEAD = 0.5;
    const double STAND_STILL_DISTANCE = Constants::ROBOT_RADIUS();
    const double MIN_OPPONENT_VELOCITY = 0.5;
    const double DEFAULT_HARASS_DISTANCE = 4 * Constants::ROBOT_RADIUS();

    const double GRID_RADIUS = 2;
    const double GRID_SIZE = 0.05;

    int tick = 0;

    struct Target {
        Vector2 targetPosition;
        int targetRobot;
    };

    enum HarassType { HARASS_OFFENSIVE, HARASS_DEFENSIVE, BLOCK_PASS, BALL, STAND_FREE };

    std::vector<world_new::view::RobotView> currentMidfielders_new;

    std::map<int, Vector2> targetPositions;
    std::map<int, world_new::view::RobotView> targetRobotsToHarass;
    std::map<int, world_new::view::RobotView> targetRobotsToHarass_new;

    Target getBall(world_new::view::RobotView &thisRobot, const world_new::view::RobotView &opponent);
    Target standFree(const Field &field, const world_new::view::RobotView &thisRobot);
    MidFieldCoach::Target harassRobot(const world_new::view::RobotView &thisRobot, const world_new::view::RobotView &opponent, HarassType harassType) const;
    Target blockPass(const world_new::view::RobotView &thisRobot, const world_new::view::RobotView &opponent, const world_new::view::BallView &ball) const;
    Vector2 calculateNewRobotPosition(const Field &field, const world_new::view::RobotView &thisRobot, Angle targetAngle);
    double calculateStandingFreeScore(const Field &field, const Vector2 &position, const world_new::view::RobotView &thisRobot);

   public:
    void addMidFielder(world_new::view::RobotView &thisRobot);
    void removeMidFielder(world_new::view::RobotView &thisRobot);
    bool validOpponent(const Field &field, const world_new::view::RobotView &opponent);

    world_new::view::RobotView findRobotToHarass(const Field &field, const world_new::view::RobotView &thisRobot);
    HarassType getHarassType(const world_new::view::RobotView &thisRobot, const world_new::view::RobotView &opponent);

    Target getTargetPosition(const Field &field, world_new::view::RobotView &thisRobot);

    HarassType getHarassTypeIfOpponentIsOnTheLeft(const world_new::view::RobotView &thisRobot, const world_new::view::BallView &ball, BallPossession &ballPossession,
                                                  const BallPossession::Possession &possession) const;

    Target &harassSlowRobot(const world_new::view::RobotView &opponent, const HarassType &harassType, Target &target) const;

    Target &harassFastRobot(const world_new::view::RobotView &thisRobot, const world_new::view::RobotView &opponent, Target &target) const;

    bool isRobotAlreadyBeingHarassed(const world_new::view::RobotView &opponent) const;
};

extern MidFieldCoach g_midFieldCoach;

}  // namespace rtt::ai::coach

#endif  // ROBOTEAM_AI_MIDFIELDCOACH_H
