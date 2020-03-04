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
#include "world_new/World.hpp"

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

    using Robot = world::Robot;
    using RobotPtr = std::shared_ptr<Robot>;
    using Ball = world::Ball;
    using BallPtr = std::shared_ptr<Ball>;
    using WorldData = world::WorldData;

    struct Target {
        Vector2 targetPosition;
        int targetRobot;
    };

    enum HarassType { HARASS_OFFENSIVE, HARASS_DEFENSIVE, BLOCK_PASS, BALL, STAND_FREE };

    std::vector<RobotPtr> currentMidfielders;
    std::vector<world_new::view::RobotView> currentMidfielders_new;

    std::map<int, Vector2> targetPositions;
    std::map<int, RobotPtr> targetRobotsToHarass;
    std::map<int, world_new::view::RobotView> targetRobotsToHarass_new;

    Target getBall(RobotPtr &thisRobot, const RobotPtr &opponent);
    Target standFree(const Field &field, const RobotPtr &thisRobot);
    MidFieldCoach::Target harassRobot(const RobotPtr &thisRobot, const RobotPtr &opponent, HarassType harassType) const;
    Target blockPass(const RobotPtr &thisRobot, const RobotPtr &opponent, const BallPtr &ball) const;
    Vector2 calculateNewRobotPosition(const Field &field, const RobotPtr &thisRobot, Angle targetAngle);
    double calculateStandingFreeScore(const Field &field, const Vector2 &position, const RobotPtr &thisRobot);

   public:
    void addMidFielder(RobotPtr &thisRobot);
    void removeMidFielder(RobotPtr &thisRobot);
    bool validOpponent(const Field &field, const RobotPtr &opponent);

    RobotPtr findRobotToHarass(const Field &field, const RobotPtr &thisRobot);
    HarassType getHarassType(const RobotPtr &thisRobot, const RobotPtr &opponent);

    Target getTargetPosition(const Field &field, RobotPtr &thisRobot);

    HarassType getHarassTypeIfOpponentIsOnTheLeft(const RobotPtr &thisRobot, const BallPtr &ball, BallPossession &ballPossession,
                                                  const BallPossession::Possession &possession) const;

    Target &harassSlowRobot(const RobotPtr &opponent, const HarassType &harassType, Target &target) const;

    Target &harassFastRobot(const RobotPtr &thisRobot, const RobotPtr &opponent, Target &target) const;

    bool isRobotAlreadyBeingHarassed(const RobotPtr &opponent) const;

    // TODO: Implement these three functions
    void addMidFielder(world_new::view::RobotView &thisRobot);
    void removeMidFielder(world_new::view::RobotView &thisRobot);
    Target getTargetPosition(const Field &field, world_new::view::RobotView &thisRobot);
    Target harassRobot(const world_new::view::RobotView &thisRobot, const world_new::view::RobotView &opponent, HarassType harassType) const;
    Target &harassSlowRobot(const world_new::view::RobotView &opponent, const MidFieldCoach::HarassType &harassType, MidFieldCoach::Target &target) const;
    bool validOpponent(const Field &field, const world_new::view::RobotView &opponent);
    world_new::view::RobotView findRobotToHarass(const Field &field, const world_new::view::RobotView &thisRobot);
    bool isRobotAlreadyBeingHarassed(const world_new::view::RobotView &opponent) const;
    HarassType getHarassType(const world_new::view::RobotView &thisRobot, const world_new::view::RobotView &opponent);
    HarassType getHarassTypeIfOpponentIsOnTheLeft(const world_new::view::RobotView &thisRobot, const world_new::view::BallView &ball, BallPossession &ballPossession, const BallPossession::Possession &possession) const;
    Target blockPass(const world_new::view::RobotView &thisRobot, const world_new::view::RobotView &opponent, const world_new::view::BallView &ball) const;
    Target getBall(world_new::view::RobotView &thisRobot, const world_new::view::RobotView &opponent);
    Target standFree(const Field &field, const world_new::view::RobotView &thisRobot);
    double calculateStandingFreeScore(const Field &field, const Vector2 &position, const world_new::view::RobotView &thisRobot);
    Target &harassFastRobot(const world_new::view::RobotView &thisRobot, const world_new::view::RobotView &opponent, MidFieldCoach::Target &target) const;
    Vector2 calculateNewRobotPosition(const Field &field, const world_new::view::RobotView &thisRobot, Angle targetAngle);

};

extern MidFieldCoach g_midFieldCoach;

}  // namespace rtt::ai::coach

#endif  // ROBOTEAM_AI_MIDFIELDCOACH_H
