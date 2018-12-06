//
// Created by baris on 16/11/18.
//


#include "ControlUtils.h"

namespace control {
double ControlUtils::calculateAngularVelocity(double robotAngle, double targetAngle) {
    double direction = 1;               // counter clockwise rotation
    double rotFactor = 8;               // how SLOW the robot rotates when it is near its destination angle

    double angleDiff = targetAngle - robotAngle;
    while (angleDiff < 0) angleDiff += 2*M_PI;
    while (angleDiff > 2*M_PI) angleDiff -= 2*M_PI;
    if (angleDiff > M_PI) {
        angleDiff = 2.0*M_PI - angleDiff;
        direction = - 1;                //  clockwise rotation
    }
    if (angleDiff > 1)angleDiff = 1;
    return direction*(std::pow(rotFactor, angleDiff - 1)*rtt::ai::constants::MAX_ANGULAR_VELOCITY - 1/rotFactor);
}
//Efficient implementation, see this: https://stackoverflow.com/questions/2049582/how-to-determine-if-a-point-is-in-a-2d-triangle
bool ControlUtils::pointInTriangle(Vec PointToCheck, Vec TP1, Vec TP2, Vec TP3) {
    double as_x = PointToCheck.x - TP1.x;
    double as_y = PointToCheck.y - TP1.y;
    bool s_ab = (TP2.x - TP1.x)*as_y - (TP2.y - TP1.y)*as_x > 0;
    if ((TP3.x - TP1.x)*as_y - (TP3.y - TP1.y)*as_x > 0 == s_ab) return false;
    return ((TP3.x - TP2.x)*(PointToCheck.y - TP2.y) - (TP3.y - TP2.y)*(PointToCheck.x - TP2.x) > 0 == s_ab);
}

double ControlUtils::TriangleArea(Vec A, Vec B, Vec C) {
    return abs((A.x*(B.y - C.y) + B.x*(C.y - A.y) + C.x*(A.y - B.y))*0.5);
}
///Square points must be connected! (e.g. SP1 is connected to SP2 and SP4)
bool ControlUtils::pointInRectangle(Vec PointToCheck, Vec SP1, Vec SP2, Vec SP3, Vec SP4) {
    if(pointInTriangle(PointToCheck,SP1,SP2,SP3)){
        return true;
    }
    else return pointInTriangle(PointToCheck,SP4,SP1,SP2);
}
double ControlUtils::constrainAngle(double angle) {
    angle= fmod(angle+M_PI,2*M_PI);
    if (angle<0)
        angle+=2*M_PI;
    return angle-M_PI;

}

//http://www.randygaul.net/2014/07/23/distance-point-to-line-segment/
//EXTENDS THE LINE!! does not take into account end and beginningpoint
double ControlUtils::distanceToLine(Vec PointToCheck, Vec LineStart,Vec LineEnd){
    Vec n=LineEnd-LineStart;
    Vec pa=LineStart-PointToCheck;
    Vec c=n * (n.dot(pa)/n.dot(n));
    Vec d= pa-c;
    return d.length();
}
double ControlUtils::distanceToLineWithEnds(Vec PointToCheck, Vec LineStart, Vec LineEnd) {
    Vec n=LineEnd-LineStart;
    Vec pa=LineStart-PointToCheck;
    Vec c=n * (n.dot(pa)/n.dot(n));
    Vec d= pa-c;
    Vec A=(PointToCheck-LineStart).project(LineStart,LineEnd);
    Vec B=(PointToCheck-LineEnd).project(LineEnd,LineStart);
    if((A.length()+B.length())>n.length()){
        return fmin(pa.length(),(LineEnd-PointToCheck).length());
    }
    else return d.length();
}

//Computes the absolute difference between 2 angles (the shortest orientation direction)
///both angles must go from[-pi,pi]!!
double ControlUtils::angleDifference(double A1, double A2){
    double angleDif=A1-A2;
    if  (angleDif<-M_PI){
        angleDif+=2*M_PI;
    }else if (angleDif>M_PI){
        angleDif-=2*M_PI;
    }
    return abs(angleDif);
}
}//control
