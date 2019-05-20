//
// Created by rolf on 15-4-19.
//

#ifndef ROBOTEAM_AI_BALLPOSSESSION_H
#define ROBOTEAM_AI_BALLPOSSESSION_H

#include "World.h"
#include "gtest/gtest_prod.h"

namespace rtt {
namespace ai {

class BallPossession {
    FRIEND_TEST(BallPossessionTest, team_far_or_close_to_ball);
public:
    enum Possession {
        LOOSEBALL,
        OURBALL,
        THEIRBALL,
        CONTENDEDBALL
    };
    void update();
    Possession getPossession();
    void recomputeState();
    std::string stateAsString(Possession state);

private:
    const double CLOSE_TIME_TRESHOLD = 0.05;//s
    const double FAR_TIME_TRESHOLD = 1.5;//s

    Possession state = LOOSEBALL;
    double closeToUsTime = 0.0;
    double closeToThemTime = 0.0;
    double farFromUsTime = 0.0;
    double farFromThemTime = 0.0;
    void updateTicks();
    bool teamCloseToBall(const world::WorldData& world, bool ourTeam);
    bool teamFarFromBall(const world::WorldData& world, bool ourTeam);
};

extern BallPossession ballPossession;
extern BallPossession* ballPossessionPtr;

} // ai
} // rtt

#endif //ROBOTEAM_AI_BALLPOSSESSION_H
