//
// Created by rolf on 3-4-19.
//

#ifndef ROBOTEAM_AI_TRUMPCOACH_H
#define ROBOTEAM_AI_TRUMPCOACH_H
///WE NEED TO BUILD A WALL
#include <roboteam_utils/Vector2.h>
namespace rtt{
namespace ai{
namespace coach{
class TrumpCoach {
    public:
        std::pair<Vector2,Vector2> getWallLine(Vector2 point,std::pair<Vector2,Vector2> goalSegment,double distance);
};

}//coach
}//ai
}//rtt

#endif //ROBOTEAM_AI_TRUMPCOACH_H
