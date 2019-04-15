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
<<<<<<< HEAD
private:
    using BallPtr = std::shared_ptr<Ball>;

    double distanceToBall;
    bool iHaveBall;
    int genevaState = 3;
    double findBallDistance(const Vector2 &ballPos);

public:

    enum Team : short {
      us,
      them
    };

    const roboteam_msgs::WorldRobot toMessage() const;
    void updateRobot(const Ball &ball);
    bool hasBall(double maxDist = Constants::MAX_BALL_BOUNCE_RANGE());
    double getDistanceToBall();
    int getGenevaState() const;
    void setGenevaState(int state);
    explicit Robot(const roboteam_msgs::WorldRobot &copy, Team team = us);
    Robot();
    int id = - 1;
    Angle angle = Angle();
    Vector2 pos = Vector2();
    Vector2 vel = Vector2();
    Vector2 acc = Vector2();
    double angularVelocity = 0.0;
    Team team;
=======
    private:
        using BallPtr = std::shared_ptr<Ball>;

        double distanceToBall;
        bool iHaveBall;
        int genevaState = 3;
public:
    int getGenevaState() const;

    void setGenevaState(int state);

private:

    double findBallDistance(const Vector2 &ballPos);

    public:

        enum Team : short {
          us,
          them
        };

        const roboteam_msgs::WorldRobot toMessage() const;
        void updateRobot(const Ball &ball);
        bool hasBall(double maxDist = Constants::MAX_BALL_BOUNCE_RANGE());
        double getDistanceToBall();
    
        explicit Robot(const roboteam_msgs::WorldRobot &copy, Team team = us);
        Robot();

        int id = - 1;
        Angle angle = Angle();
        Vector2 pos = Vector2();
        Vector2 vel = Vector2();
        Vector2 acc = Vector2();
        double angularVelocity = 0.0;
        Team team;
>>>>>>> feature/more-formations
};

} // world
} // ai
} // rtt

#endif //ROBOTEAM_AI_ROBOT_H
