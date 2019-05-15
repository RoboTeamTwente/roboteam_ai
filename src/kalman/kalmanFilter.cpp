//
// Created by kjhertenberg on 13-5-19.
//

#include "roboteam_world/kalman/kalmanFilter.h"

namespace rtt {

    kalmanFilter::kalmanFilter() {
        for (uint i = 0; i < kalmanObjectAmount; ++i) {
            kalmanlist[i] = kalmanObject(i);
        }
    }

    void kalmanFilter::kalmanUpdate() {
        for (uint i = 0; i < kalmanObjectAmount; ++i) {
            kalmanlist[i].kalmanUpdateK();
            kalmanlist[i].kalmanUpdateX();
        }
    }

    void kalmanFilter::newFrame(const roboteam_msgs::DetectionFrame msg) {
        double timeCapture = msg.t_capture;
        for (const roboteam_msgs::DetectionRobot robot : msg.them) {
            kalmanlist[robot.robot_id].kalmanUpdateZ(robot.pos.x, robot.pos.y, robot.orientation, timeCapture);
        }
        for (const roboteam_msgs::DetectionRobot robot : msg.us) {
            kalmanlist[robot.robot_id].kalmanUpdateZ(robot.pos.x, robot.pos.y, robot.orientation, timeCapture);
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

    roboteam_msgs::World kalmanFilter::getWorld(){
        roboteam_msgs::World world;
        for (uint i = 0; i < kalmanObjectAmount; ++i) {
            if (getExistence(i)) {
                if (i < 16) {
                    world.us.push_back(getRobot(i));
                }
                if (i > 16) {
                    world.them.push_back(getRobot(i));
                }
            }
        }
        return world;
    }

}