//
// Created by kjhertenberg on 13-5-19.
//

#include "roboteam_world/kalman/kalmanFilter.h"

namespace rtt {

    kalmanFilter::kalmanFilter() {
        for (uint i = 0; i < 33; ++i) {
            if (i == 32) {
                kalmanlist[i] = kalmanBall(i);
            } else if (i <= 15) {
                kalmanlist[i] = kalmanUs(i);
            } else if (i >= 16) {
                kalmanlist[i] = kalmanThem(i);
            }
        }
    }

    void kalmanFilter::kalmanUpdate() {
        for (uint i = 0; i < 33; ++i) {
            kalmanlist[i].kalmanUpdateK();
            kalmanlist[i].kalmanUpdateX();
        }
    }

    void kalmanFilter::newFrame(const roboteam_msgs::DetectionFrame msg) {
        double timeCapture = msg.t_capture;
        for (const roboteam_msgs::DetectionRobot robot : msg.us) {
            kalmanlist[robot.robot_id].kalmanUpdateZ(robot.pos.x, robot.pos.y, robot.orientation, timeCapture);
        }
        for (const roboteam_msgs::DetectionRobot robot : msg.them) {
            kalmanlist[robot.robot_id + 16].kalmanUpdateZ(robot.pos.x, robot.pos.y, robot.orientation, timeCapture);
        } for (const roboteam_msgs::DetectionBall ball : msg.balls) {
            kalmanlist[32].kalmanUpdateZ(ball.pos.x, ball.pos.y, ball.z, timeCapture);
        }
    }

    Position kalmanFilter::getPos(uint id) {
        return kalmanlist[id].kalmanGetPos();
    }

    Position kalmanFilter::getVel(uint id) {
        return kalmanlist[id].kalmanGetVel();
    }

    float kalmanFilter::getK(uint id) {
        return kalmanlist[id].getK();
    }

    void kalmanFilter::setZ(uint id, float x, float y, float z, double timestamp){
        kalmanlist[id].kalmanUpdateZ(x, y, z, timestamp);
    }

    bool kalmanFilter::getExistence(uint id) {
        return kalmanlist[id].getExistance();
    }

    roboteam_msgs::WorldRobot kalmanFilter::getRobot(uint id){
        roboteam_msgs::WorldRobot msg;
        Position pos = getPos(id);
        Position vel = getVel(id);
        msg.id = id;
        msg.pos.x = pos.x;
        msg.pos.y = pos.y;
        msg.angle = pos.rot;
        msg.vel.x = vel.x;
        msg.vel.y = vel.y;
        msg.w = vel.rot;
        return msg;
    }

    roboteam_msgs::WorldBall kalmanFilter::getBall(uint id){
        roboteam_msgs::WorldBall msg;
        Position pos = getPos(id);
        Position vel = getVel(id);
        msg.existence = 1;
        msg.visible = true;
        msg.pos.x = pos.x;
        msg.pos.y = pos.y;
        msg.z = pos.rot;
        msg.vel.x = vel.x;
        msg.vel.y = vel.y;
        msg.z_vel = vel.rot;
        return msg;
    }

    roboteam_msgs::World kalmanFilter::getWorld(){
        roboteam_msgs::World world;
        for (uint i = 0; i < 33; ++i) {
            if (getExistence(i)) {
                if (i == 32){
                    world.ball = getBall(i);
                } else if (i <= 15) {
                    world.us.push_back(getRobot(i));
                } else if (i >= 16) {
                    world.them.push_back(getRobot(i));
                }
            }
        }
        return world;
    }

}