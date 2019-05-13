//
// Created by thijs on 1-4-19.
//

#ifndef ROBOTEAM_AI_ROBOT_H
#define ROBOTEAM_AI_ROBOT_H

#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Angle.h"

#include "Ball.h"
#include <roboteam_ai/src/utilities/Constants.h>

namespace rtt {
namespace ai {
namespace world {

class Robot {
    public:
        using BallPtr = std::shared_ptr<Ball>;

    private:
        double distanceToBall;
        bool iHaveBall;
        unsigned long lastUpdatedWorldNumber = 0;

        // geneva
    private:
        unsigned char genevaState;
        unsigned char previousGenevaState = 0;
        double timeGenevaChanged = 0;
        constexpr static double timeToChangeOneGenevaState = 0.5;
        bool workingGeneva;
    public:
        unsigned char getGenevaState() const;
        bool isGenevaReady() const;
        void setGenevaState(unsigned char state = 3);
        bool hasWorkingGeneva() const;

        // dribbler
    private:
        unsigned char dribblerState;
        unsigned char previousDribblerState = 0;
        double timeDribblerChanged = 0;
        constexpr static double timeToChangeOneDribblerLevel = 0.25;
        bool workingDribbler;
    public:
        unsigned char getDribblerState() const;
        bool isDribblerReady() const;
        void setDribblerState(unsigned char dribbler = 0);
        bool hasWorkingDribbler() const;

        enum Team : short {
          us,
          them,
          invalid
        };
        explicit Robot(const roboteam_msgs::WorldRobot &copy, Team team = invalid,
                unsigned char genevaState = 3, unsigned char dribblerState = 0, unsigned long worldNumber = 0);
        Robot();

        double findBallDistance(const Vector2 &ballPos);
        const unsigned long getLastUpdatedWorldNumber() const;

        const roboteam_msgs::WorldRobot toMessage() const;
        void updateRobot(const roboteam_msgs::WorldRobot &robotMsg, const Ball &ball, unsigned long worldNumber);
        bool hasBall(double maxDist = Constants::MAX_BALL_BOUNCE_RANGE());
        double getDistanceToBall();



        int id = - 1;
        Angle angle = Angle();
        Vector2 pos = Vector2();
        Vector2 vel = Vector2();
        double angularVelocity = 0.0;
        Team team;

};

} // world
} // ai
} // rtt

#endif //ROBOTEAM_AI_ROBOT_H
