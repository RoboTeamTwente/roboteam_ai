//
// Created by kjhertenberg on 13-5-19.
//

#include "kalman/kalmanFilter.h"

void kalmanInit(){
    for (int i = 0; i < 32; ++i) {
        robotlist[i] = kalmanObject(i);
    }
}

void kalmanUpdate(){
    for (int i = 0; i < 32; ++i) {
        robotlist[i].kalmanUpdateK();
        robotlist[i].kalmanUpdateX();
    }
}

void newFrame(const roboteam_msgs::DetectionFrame msg){
    double timeCapture = msg.t_capture;
    for (const roboteam_msgs::DetectionRobot robot : msg.them) {
        robotlist(robot.robot_id).kalmanUpdateZ(robot.pos.x, robot.pos.y, robot.orientation, timeCapture)
    }
    for (const roboteam_msgs::DetectionRobot robot : msg.us) {
        robotlist(robot.robot_id).kalmanUpdateZ(robot.pos.x, robot.pos.y, robot.orientation, timeCapture)
    }
}

position getStates(uint id){
    return robotlist(id).kalmanGetState();
}