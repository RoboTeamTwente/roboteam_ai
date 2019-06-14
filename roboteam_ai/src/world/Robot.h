//
// Created by thijs on 1-4-19.
//

#ifndef ROBOTEAM_AI_ROBOT_H
#define ROBOTEAM_AI_ROBOT_H

#include <roboteam_msgs/WorldRobot.h>
#include <roboteam_utils/Vector2.h>
#include <roboteam_utils/Angle.h>
#include <gtest/gtest_prod.h>
#include "../utilities/Constants.h"
#include "Team.h"

namespace rtt {
namespace ai {

// control forward declarations
namespace control {
    class ShotController;
    class NumTreePosControl;
    class BallHandlePosControl;
    class BasicPosControl;
}

namespace world {

class Ball;
class Robot {
    FRIEND_TEST(ShotControllerTest, getshotdata_test);
public:
    using BallPtr = std::shared_ptr<Ball>;
    using RobotPtr = std::shared_ptr<Robot>;
private:
    Vector2 pidPreviousVel = Vector2();
    double distanceToBall;
    bool iHaveBall;
    unsigned long lastUpdatedWorldNumber = 0;
    int genevaState;

private:
    unsigned char dribblerState = 0;
    unsigned char previousDribblerState = 0;
    double timeDribblerChanged = 0;
    constexpr static double timeToChangeOneDribblerLevel = 0.06;

    bool workingGeneva;
    bool genevaTurning;
public:
    bool isGenevaTurning() const;

    void setGenevaTurning(bool genevaTurning);

private:
    bool workingDribbler;
    bool workingBallSensor;
    bool batteryEmpty = false;
    bool didReceiveFeedback = false;

private:

    // shotcontroller
    std::shared_ptr<control::ShotController> shotController;
    std::shared_ptr<control::NumTreePosControl> numTreePosControl;
    std::shared_ptr<control::BasicPosControl> basicPosControl;
    std::shared_ptr<control::BallHandlePosControl> ballHandlePosControl;

public:
    Robot();
    explicit Robot(const roboteam_msgs::WorldRobot &copy, Team team = invalid,
                   unsigned char genevaState = 3, unsigned char dribblerState = 0, unsigned long worldNumber = 0);
    void updateRobot(const roboteam_msgs::WorldRobot &robotMsg, const BallPtr &ball, unsigned long worldNumber);
    const unsigned long getLastUpdatedWorldNumber() const;
    int id = - 1;
    Angle angle = Angle();
    Vector2 pos = Vector2();
    Vector2 vel = Vector2();
    double angularVelocity = 0.0;
    Team team;

    double calculateDistanceToBall(const Vector2 &ballPos);
    bool hasBall(double maxDist = Constants::MAX_BALL_BOUNCE_RANGE());
    double getDistanceToBall();
    void setPidPreviousVel(const Vector2 &vel);
    const Vector2 &getPidPreviousVel() const;
    void setTimeToChangeOneGenevaState(double timeToChangeOneGenevaState);

    void setWorkingGeneva(bool workingGeneva);
    void setHasWorkingBallSensor(bool hasWorkingBallSensor);
    int getGenevaState() const;
    bool isGenevaReady() const;
    void setGenevaState(int state);
    bool hasWorkingGeneva() const;
    bool hasWorkingBallSensor() const;
    unsigned char getDribblerState() const;
    bool isDribblerReady() const;
    void setDribblerState(unsigned char dribbler = 0);
    bool hasWorkingDribbler() const;
    bool isBatteryEmpty() const;
    void setBatteryEmpty(bool batteryEmpty);

    // controllers
    const std::shared_ptr<control::ShotController> &getShotController() const;
    const std::shared_ptr<control::NumTreePosControl> &getNumtreePosControl() const;
    const std::shared_ptr<control::BasicPosControl> &getBasicPosControl() const;
    const std::shared_ptr<control::BallHandlePosControl> &getBallHandlePosControl() const;

    // reset
    void resetShotController();
    void resetNumTreePosControl();
    void resetBasicPosControl();
    void resetBallHandlePosControl();



};

} // world
} // ai
} // rtt

#endif //ROBOTEAM_AI_ROBOT_H
