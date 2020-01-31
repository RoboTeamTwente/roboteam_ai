//
// Created by mrlukasbos on 24-4-19.
//

#ifndef ROBOTEAM_AI_SHOTCONTROLLER_H
#define ROBOTEAM_AI_SHOTCONTROLLER_H

#include <control/BasicPosControl.h>
#include <control/numtrees/NumTreePosControl.h>
#include "control/RobotCommand.h"
#include "gtest/gtest_prod.h"

namespace rtt::ai::control {

enum ShotPrecision {
  LOW, // not accurate but fast
  MEDIUM, // quite accurate and quite fast
  HIGH // very accurate but slow
};

// ball speeds
enum BallSpeed {
  DRIBBLE_KICK,

  BALL_PLACEMENT,
  PASS,
  MAX_SPEED
};

class ShotController {
  FRIEND_TEST(ShotControllerTest, it_calculates_kickforce);
  FRIEND_TEST(ShotControllerTest, it_locates_robots_properly);
  FRIEND_TEST(ShotControllerTest, it_sends_proper_shoot_commands);
  FRIEND_TEST(ShotControllerTest, geneva_turning);

 private:
  bool init = false;
  bool isShooting = false;
  Vector2 aimTarget;

  // PID variables
  PID pid = PID(0.0, 0.0, 0.0);
  bool getPIDFromInterface = true;
  pidVals lastPid;
  void updatePid(pidVals pidValues);

  // Helpers
  Vector2 getPlaceBehindBallForGenevaState(const world::Robot &robot, const Vector2 &shotTarget, int genevaState);
  Vector2 getPlaceBehindBall(const world::Robot &robot,
                             const Vector2 &shotTarget); // the params are the position for the robot and the geneva angle
  int determineOptimalGenevaState(const world::Robot &robot, const Vector2 &shotTarget);
  bool onLineToBall(const world::Robot &robot, const std::pair<Vector2, Vector2> &line, ShotPrecision precision);
  bool robotAngleIsGood(world::Robot &robot, const std::pair<Vector2, Vector2> &lineToDriveOver,
                        ShotPrecision precision);
  double determineKickForce(double distance, BallSpeed desiredBallSpeed);
  std::pair<Vector2, Vector2> shiftLineForGeneva(const std::pair<Vector2, Vector2> &line, int genevaState);

  // RobotCommand calculation
  RobotCommand goToPlaceBehindBall(const Field &field, const world::Robot &robot, const Vector2 &robotTargetPosition,
                                   const std::pair<Vector2, Vector2> &driveLine, int geneva);
  RobotCommand moveStraightToBall(world::Robot robot, const std::pair<Vector2, Vector2> &lineToDriveOver);
  RobotCommand shoot(RobotCommand shotData, const world::Robot &robot, const std::pair<Vector2, Vector2> &driveLine,
                     const Vector2 &shotTarget, bool chip, BallSpeed desiredBallSpeed);
  int kickerOnTicks = 0;
  Vector2 updateGenevaAimTarget(int geneva);

 public:
  explicit ShotController() = default;
  RobotCommand getRobotCommand(const Field &field, world::Robot robot, const Vector2 &shotTarget, bool chip = false,
                               BallSpeed ballspeed = MAX_SPEED, bool useAutoGeneva = true, ShotPrecision precision = MEDIUM, int genevaState = 0);
};

}  // namespace rtt::ai::control
#endif  // ROBOTEAM_AI_SHOTCONTROLLER_H