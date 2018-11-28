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
bool ControlUtils::pointInTriangle(Vector2 PointToCheck, Vector2 TP1, Vector2 TP2, Vector2 TP3) {
    double as_x = PointToCheck.x - TP1.x;
    double as_y = PointToCheck.y - TP1.y;
    bool s_ab = (TP2.x - TP1.x)*as_y - (TP2.y - TP1.y)*as_x > 0;
    if ((TP3.x - TP1.x)*as_y - (TP3.y - TP1.y)*as_x > 0 == s_ab) return false;
    return (TP3.x - TP2.x)*(PointToCheck.y - TP2.y) - (TP3.y - TP2.y)*(PointToCheck.x - TP2.x) > 0 == s_ab;
}

double ControlUtils::TriangleArea(Vector2 A, Vector2 B, Vector2 C) {
    return abs((A.x*(B.y - C.y) + B.x*(C.y - A.y) + C.x*(A.y - B.y))/2.0);
}
//https://www.geeksforgeeks.org/check-whether-given-point-lies-inside-rectangle-not/
///Square points must be connected! (e.g. SP1 is connected to SP2 and SP4)
bool ControlUtils::pointInRectangle(Vector2 PointToCheck, Vector2 SP1, Vector2 SP2, Vector2 SP3, Vector2 SP4) {
    double A = TriangleArea(SP1, SP2, SP3) + TriangleArea(SP1, SP4, SP3);
    double A1 = TriangleArea(PointToCheck, SP1, SP2);
    double A2 = TriangleArea(PointToCheck, SP2, SP3);
    double A3 = TriangleArea(PointToCheck, SP3, SP4);
    double A4 = TriangleArea(PointToCheck, SP1, SP4);
    return (A == A1 + A2 + A3 + A4);
}