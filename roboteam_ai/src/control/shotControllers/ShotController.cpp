//
// Created by mrlukasbos on 24-4-19.
//

#include <roboteam_ai/src/world/World.h>
#include <roboteam_ai/src/control/ControlUtils.h>
#include <roboteam_ai/src/control/PositionUtils.h>
#include <roboteam_ai/src/interface/api/Input.h>
#include "ShotController.h"

namespace rtt {
namespace ai {
namespace control {
ShotController::ShotController(ShotPrecision precision, BallSpeed ballspeed, bool useAutoGeneva)
        : precision(precision), ballSpeed(ballspeed), useAutoGeneva(useAutoGeneva) {
    numTreeGtp.setAvoidBall(Constants::BALL_RADIUS() + Constants::ROBOT_RADIUS() * 1.5);
    numTreeGtp.setCanMoveOutOfField(true);
    numTreeGtp.setCanMoveInDefenseArea(false);
}

/// return a ShotData (which contains data for robotcommands) for a specific robot to shoot at a specific target.
ShotData ShotController::getShotData(world::Robot robot, Vector2 shotTarget, bool chip) {
    auto ball = world::world->getBall();

    // only get a new geneva state if we are allowed to get one
    bool robotAlreadyVeryClose = robot.pos.dist(ball->pos) < 3.0 * Constants::ROBOT_RADIUS();
    int currentDesiredGeneva = robot.getGenevaState();

    if (useAutoGeneva && robot.hasWorkingGeneva && !genevaIsTurning && !robotAlreadyVeryClose) {
        currentDesiredGeneva = determineOptimalGenevaState(robot, shotTarget);
    }

    if (chip) {
        currentDesiredGeneva = 3;
    }

    behindBallPosition = ball->pos + getPlaceBehindBallForGenevaState(robot, shotTarget, currentDesiredGeneva);

    // make a line, on which we can drive straight to it

    std::pair<Vector2, Vector2> lineToDriveOver = std::make_pair(behindBallPosition, ball->pos);
        /*
        std::cout<<" current diff:  "<<abs(robot.angle.getAngle()-(lineToDriveOver.second-lineToDriveOver.first).angle());
        std::cout<<" drive: " <<(lineToDriveOver.second-lineToDriveOver.first).angle();
        std::cout<<"  geneva: " <<currentDesiredGeneva;
        std::cout<<"  goalAngle: "<<(shotTarget-ball->pos).angle();
        */
    // check the properties
    bool isOnLineToBall = onLineToBall(robot, lineToDriveOver);
    bool isBehindBall = control::PositionUtils::isRobotBehindBallToPosition(0.80, shotTarget, robot.pos, 0.3);
    bool validAngle = robotAngleIsGood(robot, lineToDriveOver);

    std::cout << isOnLineToBall << isBehindBall << validAngle << std::endl;

    ShotData shotData;
    if (isOnLineToBall && isBehindBall && validAngle) {
        bool hasBall = world::world->ourRobotHasBall(robot.id, Constants::MAX_KICK_RANGE());
        if (hasBall && !genevaIsTurning) {
            shotData = shoot(robot, lineToDriveOver, shotTarget, chip);
          //  std::cout<<" SHOOT";
        } else if (hasBall && genevaIsTurning) {
            // just stand still at the right angle
            shotData.vel = {0.0, 0.0};
            shotData.angle = (lineToDriveOver.second - lineToDriveOver.first).angle();
            std::cout << "Not shooting because geneva is turning for " << secondsToTurnGeneva << "s" << std::endl;
        } else {
            shotData = moveStraightToBall(robot, lineToDriveOver);
            //std::cout<<" MOVING";
        }
    } else {
        shotData = goToPlaceBehindBall(robot, behindBallPosition, lineToDriveOver);
       // std::cout<<" GOING BEHIND";
    }

    interface::Input::drawData(interface::Visual::SHOTLINES, {ball->pos, shotTarget}, Qt::blue, robot.id, interface::Drawing::LINES_CONNECTED);

    // Make sure the Geneva state is always correct
    shotData.genevaState = currentDesiredGeneva;
    return shotData;
}


void ShotController::setGenevaDelay(int genevaDifference) {
    if (genevaDifference != 0) {
        genevaIsTurning = true;
        // each turn should increase the time which the geneva is turning
        secondsToTurnGeneva = genevaDifference * 0.4;
        lastTimeGenevaChanged = ros::Time::now().toSec();
    }
}

/// check if a robot is on a line to a ball

bool ShotController::onLineToBall(const world::Robot &robot, std::pair<Vector2, Vector2> line) {
    double dist = ControlUtils::distanceToLine(robot.pos, line.first, line.second);
    //std::cout<<dist<<" ";
    if (precision == HIGH) {
        return dist < 0.1;
    } else if (precision == MEDIUM) {
        return dist < 0.15;
    }
    return dist < 0.15;
}

/// return the place behind the ball targeted towards the ball target position
    Vector2 ShotController::getPlaceBehindBall(world::Robot robot, Vector2 shotTarget) {
        auto ball = world::world->getBall();
        Vector2 preferredShotVector = ball->pos - shotTarget;
        double distanceBehindBall = 2.0 * Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();
        return ball->pos + preferredShotVector.stretchToLength(distanceBehindBall);
    }

// use Numtree GTP to go to a place behind the ball
    ShotData ShotController::goToPlaceBehindBall(world::Robot robot, Vector2 robotTargetPosition,
                                                 std::pair<Vector2, Vector2> line) {
    std::cout << "Go behind ball" << std::endl;
    std::cout << (robot.pos - robotTargetPosition).length() << std::endl;
        auto ball = world::world->getBall();
        control::PosVelAngle pva = numTreeGtp.getPosVelAngle(std::make_shared<world::Robot>(robot),
                                                             robotTargetPosition);
        //TODO: if (rotating to this angle from current angle will hit ball) then pva.angle=angle towards ball
        if ((robot.pos - robotTargetPosition).length() < 0.3) {
            pva.angle = (line.second - line.first).toAngle();
        }
        if (pva.vel.length()<0.3)
        {
            pva.vel.stretchToLength(0.3);
        }
        ShotData shotData(pva);
        return shotData;
    }

/// At this point we should be behind the ball. now we can move towards the ball to kick it.
ShotData ShotController::moveStraightToBall(world::Robot robot, std::pair<Vector2, Vector2> lineToDriveOver) {
    std::cout << "Moving straight to ball" << std::endl;
    control::PosVelAngle pva = basicGtp.getPosVelAngle(std::make_shared<world::Robot>(robot), lineToDriveOver.second);
    pva.angle = (lineToDriveOver.second - lineToDriveOver.first).toAngle();
    ShotData shotData(pva);
    return shotData;
}

/// Now we should have the ball and kick it.
    ShotData ShotController::shoot(world::Robot robot, std::pair<Vector2, Vector2> driveLine, Vector2 shotTarget,
                                   bool chip) {
    std::cout << "Shooting" << std::endl;
        auto ball = world::world->getBall();

        // move towards the ball
        control::PosVelAngle pva = basicGtp.getPosVelAngle(std::make_shared<world::Robot>(robot), driveLine.second);
        if (pva.vel.length() < 0.3) {
            pva.vel = pva.vel.stretchToLength(0.3);
        }
        pva.angle = (driveLine.second - driveLine.first).toAngle();

        ShotData shotData(pva);

        // set the kicker and kickforce
        if (chip) {
            shotData.chip = true;
            shotData.kick = false;

            // TODO calibrate chip speed
            shotData.kickSpeed = determineKickForce(ball->pos.dist(shotTarget));
        } else {
            shotData.chip = false;
            shotData.kick = true;
            shotData.kickSpeed = determineKickForce(ball->pos.dist(shotTarget));
        }
        return shotData;
    }

/// Determine how fast we should kick for a pass at a given distance
    double ShotController::determineKickForce(double distance) {
        const double maxPowerDist = rtt::ai::Constants::MAX_POWER_KICK_DISTANCE();

        double velocity = 0;
        switch (ballSpeed) {
            case DRIBBLE_KICK:
                velocity = sqrt(distance) * rtt::ai::Constants::MAX_KICK_POWER() / (sqrt(maxPowerDist) * 1.5);
                break;
            case LAY_STILL_AT_POSITION:
                velocity = sqrt(distance) * rtt::ai::Constants::MAX_KICK_POWER() / (sqrt(maxPowerDist) * 1.5);
                break;
            case PASS:
                velocity = distance > maxPowerDist ? rtt::ai::Constants::MAX_KICK_POWER() : sqrt(distance)
                                                                                            *
                                                                                            rtt::ai::Constants::MAX_KICK_POWER() /
                                                                                            sqrt(maxPowerDist) *
                                                                                            1.2;
                break;
            case MAX_SPEED:
                velocity = rtt::ai::Constants::MAX_KICK_POWER();
                break;
        }

        // limit the output to the max kick speed
        return std::min(velocity, rtt::ai::Constants::MAX_KICK_POWER());
    }

    void ShotController::makeCommand(ShotData data, roboteam_msgs::RobotCommand &command) {
        command.x_vel = data.vel.x;
        command.y_vel = data.vel.y;
        command.w = data.angle.getAngle();
        command.chipper = data.chip;
        command.chipper_vel = data.kickSpeed;
        command.kicker = data.kick;
        command.kicker_forced = data.kick;
        command.kicker_vel = data.kickSpeed;
        command.geneva_state = data.genevaState;
    }

    bool ShotController::robotAngleIsGood(world::Robot &robot,
                                          std::pair<Vector2, Vector2> lineToDriveOver) {
        Angle aim((lineToDriveOver.second - lineToDriveOver.first).toAngle());
        double diff = abs(aim - robot.angle);
        if (precision == HIGH) {
            return diff < 0.05;
        }
        if (precision == MEDIUM) {
            return diff < 0.2;
        }
        return diff < 0.5;
    }


// get the place behind the ball as if no geneva is used
// we rotate this vector according to the angle with the shotline
// we intentionally need to remove ball pos because we are rotating the vector
Vector2
ShotController::getPlaceBehindBallForGenevaState(world::Robot robot, Vector2 shotTarget, int genevaState) {
    auto ball = world::world->getBall();
    Vector2 placeStraightBehindBallVector = getPlaceBehindBall(robot, shotTarget) - ball->pos;

    switch (genevaState) {
        case 1:
            return placeStraightBehindBallVector.rotate(toRadians(20));
        case 2:
            return placeStraightBehindBallVector.rotate(toRadians(10));
        case 3:
            return placeStraightBehindBallVector;
        case 4:
            return placeStraightBehindBallVector.rotate(-toRadians(10));
        case 5:
            return placeStraightBehindBallVector.rotate(-toRadians(20));
    }
}

int ShotController::determineOptimalGenevaState(world::Robot robot, Vector2 shotTarget) {
    auto ball = world::world->getBall();

    // determine the shortest position from where to kick the ball
    Vector2 robotToBall = ball->pos - robot.pos;
    Vector2 preferredShotVector = shotTarget - ball->pos;

    // determine the angle between the robot position and the shotline
    Angle angleWithShotline = robotToBall.toAngle() - preferredShotVector.toAngle();
    if (angleWithShotline.getAngle() > toRadians(20)) {
        return 2;
    } else if (angleWithShotline.getAngle() < (- toRadians(20))) {
        return 4;
    }
    return 3;
}

}// control
} // ai
} // rtt