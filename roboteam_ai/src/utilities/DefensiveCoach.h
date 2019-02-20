//
// Created by rolf on 18-2-19.
//

#ifndef ROBOTEAM_AI_DEFENSIVECOACH_H
#define ROBOTEAM_AI_DEFENSIVECOACH_H
#include "roboteam_utils/Vector2.h"
#include "../utilities/World.h"
namespace rtt{
namespace ai{
namespace coach{
class DefensiveCoach {
    private:
        //BLOCKPOS actively covers the goal up from the ball or robots who can receive the ball and score
        //PASSIVEINTERCEPTPOS tries to go to a position where in case of a pass to another robot it can intercept it in case of a pass
        //ACTIVEINTERCEPTPOS is actively blocking intercepts to other robots
        enum PosType{BLOCKPOS, PASSIVEINTERCEPTPOS};
        std::map<int,Vector2> robotWithPos;
        Vector2 computeBlockPos();
        Vector2 computePassiveInterceptPos();
        Vector2 computeActiveInterceptPos();
        //pass detection
        bool theyCanPass();
        std::vector<std::pair<int,double>> possiblePassTargets();

    public:
};

}
}
}


#endif //ROBOTEAM_AI_DEFENSIVECOACH_H
