//
// Created by mrlukasbos on 5-10-18.
// Used internally to accumulate data.

#ifndef ROBOTEAM_AI_PARTIALRESULT_H
#define ROBOTEAM_AI_PARTIALRESULT_H

typedef unsigned char DangerFlag;

constexpr DangerFlag DANGER_NONE = 0b00000000; //< Placeholder for empty flags
constexpr DangerFlag DANGER_FREE = 0b00000001; //< The robot could receive the ball easily
constexpr DangerFlag DANGER_CLOSING = 0b00000010; //< The robot is closing on our goal
constexpr DangerFlag DANGER_CAN_SHOOT = 0b00000100; //< The robot has the ball and could shoot at our goal
constexpr DangerFlag DANGER_CAN_CROSS = 0b00001000; //< The robot has the ball and could pass it to another opponent near our goal.
constexpr DangerFlag DANGER_HAS_BALL = 0b00010000; //< The robot has the ball
constexpr DangerFlag DANGER_IS_GOALIE = 0b00100000; //< The robot is probably the opponents' keeper

struct PartialResult {
  double score;
  DangerFlag flags;
  PartialResult();
  PartialResult(double score, DangerFlag flags);
  PartialResult &operator+=(const PartialResult &b);
};

PartialResult operator+(PartialResult a, PartialResult b);

#endif //ROBOTEAM_AI_PARTIALRESULT_H
