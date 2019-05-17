//
// Created by kjhertenberg on 13-5-19.
//

#include "roboteam_world/kalman/kalmanFilter.h"

namespace rtt {

    kalmanFilter::kalmanFilter() {
        for (uint i = 0; i <TOTALCOUNT; ++i) {
            if (i < BOTCOUNT) {
                kalmanlist[i] = kalmanUs(i);
            } else if (i < BOTCOUNT*2) {
                kalmanlist[i] = kalmanThem(i);
            }
            else {
                kalmanlist[i] = kalmanBall();
            }
        }
    }

    void kalmanFilter::kalmanUpdate() {
        for (uint i = 0; i < TOTALCOUNT; ++i) {
            kalmanlist[i].kalmanUpdateK();
            kalmanlist[i].kalmanUpdateX();
        }
    }

    // if we get a new frame we update our observations
    void kalmanFilter::newFrame(const roboteam_msgs::DetectionFrame& msg) {
        double timeCapture = msg.t_capture;
        lastFrameTime=timeCapture;
        for (const roboteam_msgs::DetectionRobot robot : msg.us) {
            kalmanlist[robot.robot_id].kalmanUpdateZ(robot.pos.x, robot.pos.y, robot.orientation, timeCapture);
        }
        for (const roboteam_msgs::DetectionRobot robot : msg.them) {
            kalmanlist[robot.robot_id + BOTCOUNT].kalmanUpdateZ(robot.pos.x, robot.pos.y, robot.orientation, timeCapture);
        } for (const roboteam_msgs::DetectionBall ball : msg.balls) {
            kalmanlist[2*BOTCOUNT].kalmanUpdateZ(ball.pos.x, ball.pos.y, ball.z, timeCapture);
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
        return kalmanlist[id].getExistence();
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
        std::cout<<msg.z<<" (z):"<<msg.z_vel<<" (zvel):"<<std::endl;
        return msg;
    }

    roboteam_msgs::World kalmanFilter::getWorld(){
        roboteam_msgs::World world;
        world.time=lastFrameTime;
        for (uint i = 0; i < TOTALCOUNT; ++i) {
            if (getExistence(i)) {
                if (i < BOTCOUNT) {
                    world.us.push_back(getRobot(i));
                } else if (i<2*BOTCOUNT) {
                    world.them.push_back(getRobot(i));
                }
                else{
                    world.ball=getBall(i);
                }
            }
        }
        return world;
    }

}