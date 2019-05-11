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

// control forward declarations
namespace control{
    class ShotController;
    class NumTreePosControl;
    class BasicPosControl;
}

namespace world {

class Robot {
private:
    using BallPtr = std::shared_ptr<Ball>;
    double distanceToBall;
    bool iHaveBall;
    static std::map<int, unsigned char> genevaState;
    unsigned long lastUpdatedWorldNumber = 0;

    // control managers
    std::shared_ptr<control::NumTreePosControl> numtreeGTP;
    std::shared_ptr<control::BasicPosControl> basicGTP;
    std::shared_ptr<control::ShotController> shotController;

public:

    enum Team : short {
      us,
      them,
      invalid
    };

    bool hasWorkingGeneva = false;
    int id = - 1;
    Angle angle = Angle();
    Vector2 pos = Vector2();
    Vector2 vel = Vector2();
    double angularVelocity = 0.0;
    Team team;

    explicit Robot(const roboteam_msgs::WorldRobot &copy, Team team = invalid, unsigned long worldNumber = 0);
    Robot();
    double findBallDistance(const Vector2 &ballPos);
    const roboteam_msgs::WorldRobot toMessage() const;
    void updateRobot(roboteam_msgs::WorldRobot robotMsg, const Ball &ball, unsigned long worldNumber);
    bool hasBall(double maxDist = Constants::MAX_BALL_BOUNCE_RANGE());
    double getDistanceToBall();
    unsigned char getGenevaState() const;
    void setGenevaState(unsigned char state = 3);
    const unsigned long &getLastUpdatedWorldNumber() const;
    const std::shared_ptr<control::ShotController> &getShotController() const;
    const std::shared_ptr<control::NumTreePosControl> &getNumtreeGtp() const;
    const std::shared_ptr<control::BasicPosControl> &getBasicGtp() const;
};

} // world
} // ai
} // rtt

#endif //ROBOTEAM_AI_ROBOT_H
