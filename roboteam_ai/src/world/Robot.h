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
    private:
        using BallPtr = std::shared_ptr<Ball>;

        double distanceToBall;
        bool iHaveBall;
        static std::map<int, unsigned char> genevaState;
        unsigned long lastUpdatedWorldNumber = 0;
public:
    const unsigned long &getLastUpdatedWorldNumber() const;

public:
        enum Team : short {
          us,
          them,
          invalid
        };
    bool hasWorkingGeneva = false;

        explicit Robot(const roboteam_msgs::WorldRobot &copy, Team team = invalid, unsigned long worldNumber = 0);
        Robot();

        double findBallDistance(const Vector2 &ballPos);

        const roboteam_msgs::WorldRobot toMessage() const;
        void updateRobot(const roboteam_msgs::WorldRobot robotMsg, const Ball &ball, unsigned long worldNumber);
        bool hasBall(double maxDist = Constants::MAX_BALL_BOUNCE_RANGE());
        double getDistanceToBall();
        unsigned char getGenevaState() const;
        void setGenevaState(unsigned char state = 3);

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
