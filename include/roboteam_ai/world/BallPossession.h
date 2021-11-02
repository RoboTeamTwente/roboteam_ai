#ifndef ROBOTEAM_AI_BALLPOSSESSION_H
#define ROBOTEAM_AI_BALLPOSSESSION_H

#include <gtest/gtest_prod.h>

#include "Field.h"
#include "world/views/WorldDataView.hpp"

namespace rtt::ai {

/**
 * Computes and stores the ball possession state which indicates which team controls the ball (can also be both/no team).
 * @author Created by: Rolf van der Hulst <br>
 *         Documented by: Haico Dorenbos
 * @since 2019-04-15
 */
class BallPossession {
   private:
    unsigned long previousTime = 0;
    const double MIDDLE_LINE_X = 0.0;  // The x coordinate that corresponds to the middle of the field.
    /* If a teams distance to the ball is larger or equal than STANDARD_FAR_THRESHOLD at a moment then that team is
     * considered to be far from the ball at that moment. */
    const double STANDARD_FAR_THRESHOLD = 0.4;
    /* Use DEFENSIVE_FAR_THRESHOLD instead of STANDARD_FAR_THRESHOLD for the opponents team if the ball is at our side
     * of the field. In this case we should play more defensively and therefore the opponent is only considered to be
     * far from the ball if the opponent has a larger distance to the ball than in the general case. */
    const double DEFENSIVE_FAR_THRESHOLD = 0.9;
    /* If the ball has been uninterruptedly close to a team for this number of seconds then this team is considered to
     * be long enough close to the ball. */
    const double CLOSE_TIME_TRESHOLD = 0.05;
    /* If the ball has been uninterruptedly far from a team for this number of seconds then this team is considered
     * to be long enough far from the ball. */
    const double FAR_TIME_TRESHOLD = 1.5;
    /* If the ball x coordinate is larger than relatively OUR_POSSESSION_RELATIVE_LENGTH_THRESHOLD of the length of the
     * field (with respect to the leftmost x-coordinate of the field) then the ball is considered to be always in our
     * possession. */
    const double OUR_POSSESSION_RELATIVE_LENGTH_THRESHOLD = 0.75;  // 6/8 of the field
    /* If the ball x coordinate is smaller than relatively THEIR_POSSESSION_RELATIVE_LENGTH_THRESHOLD of the length of
     * the field (with respect to the leftmost x-coordinate of the field) then the ball is always considered to be in
     * their possession. */
    const double THEIR_POSSESSION_RELATIVE_LENGTH_THRESHOLD = 0.375;  // 3/8 of the field

   public:
    enum Possession {
        LOOSEBALL,     // If neither us nor the opponent possess the ball.
        OURBALL,       // If we possess the ball.
        THEIRBALL,     // If the opponent possess the ball.
        CONTENDEDBALL  // If we and the opponent both possess the ball.
    };

    /**
     * Runs every tick to update which team possess the ball (can also be both/no team).
     */
    void update(world::view::WorldDataView world, const world::Field &field);

    /**
     * Check which team possess the ball (can also be both/no team).
     * @return Which team possess the ball (can also be both/no team).
     */
    Possession getPossession();

   private:
    Possession state = LOOSEBALL;

    double closeToUsTime = 0.0;    // For how many seconds uninterrupted our team has remained close to the ball.
    double closeToThemTime = 0.0;  // For how many seconds uninterrupted the opponents team has remained close to the ball.
    double farFromUsTime = 0.0;    // For how many seconds uninterrupted our team has remained far from the ball.
    double farFromThemTime = 0.0;  // For how many seconds uninterrupted the opponents team has remained far from the ball.

    /**
     * Updates all close and far away times (for how many seconds uninterrupted each team has remained close/far from
     * the ball).
     */
    void updateCloseAndFarTimes(world::view::WorldDataView world);

    /**
     * Checks if a team is currently (at this moment) relatively close to the ball. Returns true if that team is close
     * to the ball, returns false if that team is not close to the ball.
     */
    bool teamCloseToBall(world::view::WorldDataView world, bool ourTeam);

    /**
     * Checks if a team is currently (at this moment) relatively far from the ball. Returns true if that team is far
     * away from the ball, returns false if that team is not far away from the ball.
     */
    bool teamFarFromBall(world::view::WorldDataView world, bool ourTeam);

    /**
     * Recompute which team possess the ball (can also be both/no team).
     */
    void recomputeState(world::view::WorldDataView world, const world::Field &field);
};

extern BallPossession ballPossession;
extern BallPossession *ballPossessionPtr;

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_BALLPOSSESSION_H
