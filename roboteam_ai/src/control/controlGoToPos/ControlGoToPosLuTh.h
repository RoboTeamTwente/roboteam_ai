//
// Created by thijs on 12-12-18.
//

#include "goToPosInclude.h"

#ifndef ROBOTEAM_AI_CONTROLGOTOPOSLUTH_H
#define ROBOTEAM_AI_CONTROLGOTOPOSLUTH_H

namespace control {

class ControlGoToPosLuTh {

    private:
        using RobotPtr = std::shared_ptr<roboteam_msgs::WorldRobot>;
        using Vector2 = rtt::Vector2;
        using Command = roboteam_msgs::RobotCommand;

        struct NumRobot;
        using NumRobotPtr = std::shared_ptr<NumRobot>;
        struct NumRobot {

          int id = - 1;                       //Robot id
          unsigned long startIndex = 0;
          Vector2 pos;                  //Current x,y position in m
          Vector2 targetPos;            //Target position in m
          Vector2 finalTargetPos;
          Vector2 vel;                  //Current x,y velocity in ms-1
          Vector2 targetVel;            //Target velocity in ms-1
          double maxVel = 2.5;          //Maximum velocity in ms-1
          Vector2 acc;                  //Current x,y acceleration in ms-2
          double maxAcc = 2.5;          //Maximum acceleration in ms-2
          std::vector<Vector2> posData = {{}}; //Save the position data
          std::vector<Vector2> velData = {{}}; //Save the velocity data
          float t = 0;
          const float dt = 0.05f;
          int totalCalculations = 0;
          int collisions = 0;

          enum newDirections {
            goLeft,
            goMiddle,
            goRight
          };
          newDirections newDir = goMiddle;

          Vector2 getDirection() {
              return (targetPos - pos).normalize();
          }

          Vector2 getDirection(Vector2 &target) {
              return (target - pos).normalize();
          }

          bool isCollision(Vector2 &otherPos) {
              double minDistance = 0.3;
              return isCollision(otherPos, minDistance);
          }

          bool isCollision(Vector2 &otherPos, double minDistance) {
              return (std::abs((otherPos - pos).length()) < minDistance);
          }

          std::vector<Vector2> getNewTargetsOLD(Vector2 &collisionPos, Vector2 &startPos) {
              std::vector<Vector2> newTargets;

              Vector2 deltaPos = collisionPos - startPos;
              int maxI = 4; //(int) ceil(8.0f/(me->collisions + 1));
              for (int i = 1 - maxI; i < maxI; i ++) {
                  auto angle = (double) abs(i)*i*M_PI/(maxI*maxI);
                  Vector2 newDeltaPos = deltaPos.rotate(angle)*(1 + M_PI - abs(angle))/(M_PI);
                  Vector2 newTarget = (collisionPos + startPos)*0.5 + newDeltaPos*0.5;
                  newTargets.push_back(newTarget);
              }
              return newTargets;
          }

          std::vector<std::pair<newDirections, Vector2>> getNewTargets(Vector2 &collisionPos, Vector2 &startPos) {
              std::vector<std::pair<newDirections, Vector2>> newTargets;

              Vector2 deltaPos = collisionPos - startPos;
              std::vector<double> angles;
              switch (newDir) {
              case goLeft: {
                  angles = { - 3*M_PI*0.0625, - 2*M_PI*0.0625, - M_PI*0.0625};
                  break;
              }
              case goMiddle: {
                  angles = {- M_PI*0.0625, M_PI*0.0625};
                  break;
              }
              case goRight: {
                  angles = {M_PI*0.0625, 2*M_PI*0.0625, 4*M_PI*0.0625};
                  break;
              }
              }

              for (double angle : angles) {

                  if (angle < 0) newDir = goLeft;
                  else if (angle > 0) newDir = goRight;
                  else newDir = goMiddle;

                  Vector2 newTarget = startPos + deltaPos.rotate(angle);
                  newTargets.push_back({newDir, newTarget});
              }
              return newTargets;
          }

          NumRobotPtr getNewNumRobot(NumRobotPtr me, std::pair<newDirections, Vector2> &newTarget) {
              NumRobot newMe;

              std::vector<Vector2> _posData(me->posData.begin(), me->posData.begin() + me->startIndex);
              newMe.posData = _posData;
              newMe.pos = newMe.posData.back();//me->posData[me->startIndex];
              std::vector<Vector2> _velData(me->velData.begin(), me->velData.begin() + me->startIndex);
              newMe.velData = _velData;
              newMe.vel = newMe.velData.back();//me->velData[me->startIndex];

              newMe.id = me->id;
              newMe.totalCalculations = me->totalCalculations;
              newMe.collisions = me->collisions + 1;
              newMe.startIndex = newMe.posData.size() - 1;
              newMe.targetPos = newTarget.second;
              newMe.newDir = newTarget.first;
              newMe.finalTargetPos = me->finalTargetPos;
              if (newMe.pos.length() < 0.1) {
                  std::cout << "errorr??" << std::endl;
              }
              return std::make_shared<NumRobot>(newMe);
          }

          void clear() {
              posData.clear();
              velData.clear();
              pos = {0, 0};
              vel = {0, 0};
              acc = {0, 0};
              finalTargetPos = {0, 0};
              targetPos = {0, 0};

              NumRobot newMe;

              id = newMe.id;
              startIndex = newMe.startIndex;
              maxVel = newMe.maxVel;
              maxAcc = newMe.maxAcc;
              t = newMe.t;
              totalCalculations = newMe.totalCalculations;
              collisions = newMe.collisions;
          }

          struct CustomCompare {
            bool operator()(NumRobotPtr lhs, NumRobotPtr rhs) {
                if (lhs->collisions < rhs->collisions) return false;
                else
                    return abs((lhs->pos - lhs->finalTargetPos).length())
                            > abs((rhs->pos - rhs->finalTargetPos).length());
            }
          };

        };

        std::priority_queue<NumRobotPtr, std::vector<NumRobotPtr>, NumRobot::CustomCompare> robotQueue;
        NumRobot me;

        std::vector<Vector2> displayData;
        double errorMargin = 0.3;

        Vector2 targetPos = {0,0};
        ros::Time startTime;

        PID pid;
        bool pidInit = false;

        bool tracePath(NumRobot &numRobot, Vector2 target);
        bool calculateNumericDirection(RobotPtr robot, NumRobot &me, roboteam_msgs::RobotCommand &command);
        void drawCross(Vector2 &pos);
        bool calculateNextPoint(NumRobotPtr me);
        int robotIndex;
    public:
        void clear();
        Command goToPos(RobotPtr robot, Vector2 &targetPos);
};

} // control

#endif //ROBOTEAM_AI_CONTROLGOTOPOSLUTH_H