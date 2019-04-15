//
// Created by rolf on 15-4-19.
//

#ifndef ROBOTEAM_AI_BALLPOSSESSION_H
#define ROBOTEAM_AI_BALLPOSSESSION_H

#include "World.h"
namespace rtt {
namespace ai {
class BallPossession {
    public:
        enum Possession {LOOSEBALL, OURBALL, THEIRBALL, CONTENDEDBALL };
    private:
        Possession state=LOOSEBALL;
        double closeToUsTime=0.0,closeToThemTime=0.0,farFromUsTime=0.0,farFromThemTime=0.0;
        void updateTicks();
        bool teamCloseToBall(const world::WorldData& world, bool ourTeam);
        bool teamFarFromBall(const world::WorldData& world, bool ourTeam);

    public:
        void update();
        Possession getPossession();
        void recomputeState();
        std::string stateAsString();

};
extern BallPossession bpTrackerObj;
extern BallPossession* bpTracker;
}
}

#endif //ROBOTEAM_AI_BALLPOSSESSION_H
