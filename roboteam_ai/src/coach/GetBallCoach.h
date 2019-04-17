//
// Created by rolf on 17-4-19.
//

#ifndef ROBOTEAM_AI_GETBALLCOACH_H
#define ROBOTEAM_AI_GETBALLCOACH_H
namespace rtt{
namespace ai{
namespace coach{
class GetBallCoach {
    private:
        bool gettingBall=false;
        int idGettingBall=-1;
        bool shouldWeGetBall();
        int bestBallGetterID();
    public:
        void update();
        bool weAreGettingBall();
        int getBallGetterID();



};

extern GetBallCoach GBCoachObj;
extern GetBallCoach* GBCoach;

}
}
}

#endif //ROBOTEAM_AI_GETBALLCOACH_H
