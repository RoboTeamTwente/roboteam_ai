//
// Created by thijs on 12-12-18.
//

#include "GoToPosInclude.h"
#include <roboteam_ai/src/interface/InterfaceValues.h>

#ifndef ROBOTEAM_AI_CONTROLGOTOPOSLUTH_H
#define ROBOTEAM_AI_CONTROLGOTOPOSLUTH_H

namespace rtt {
namespace ai {
namespace control {

class ControlGoToPosLuTh {

    private:
        using RobotPtr = std::shared_ptr<roboteam_msgs::WorldRobot>;

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
          double maxVel = 1.56;          //Maximum velocity in ms-1
          Vector2 acc;                  //Current x,y acceleration in ms-2
          double maxAcc = 3.03;          //Maximum acceleration in ms-2
          double defaultCollisionRadius = 0.25;
          std::vector<Vector2> posData = {{}}; //Save the position data
          std::vector<Vector2> velData = {{}}; //Save the velocity data
          float t = 0;
          const float dt = 0.0175f;
          int totalCalculations = 0;
          int collisions = 0;
          bool careAboutFieldEdge = true;

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
              return isCollision(otherPos, defaultCollisionRadius);
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
              double deltaAngle = 0.055;
              switch (newDir) {
              case goLeft: {
                  angles = {- M_PI*deltaAngle};
                  break;
              }
              case goMiddle: {
                  angles = {- M_PI*deltaAngle, M_PI*deltaAngle};
                  break;
              }
              case goRight: {
                  angles = {M_PI*deltaAngle};
                  break;
              }
              }

              for (double angle : angles) {

                  if (angle > 0) newDir = goLeft;
                  else if (angle < 0) newDir = goRight;
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
              if (newMe.posData.size() != 0) {
                  newMe.startIndex = newMe.posData.size() - 1;
                  newMe.pos = newMe.posData.back();
              }
              else {
                  newMe.startIndex = 0;
                  newMe.pos = World::getRobotForId(static_cast<unsigned int>(me->id), true).get()->pos;
              }
              std::vector<Vector2> _velData(me->velData.begin(), me->velData.begin() + me->startIndex);
              newMe.velData = _velData;
              newMe.vel = newMe.velData.back();

              newMe.id = me->id;
              newMe.totalCalculations = me->totalCalculations;
              newMe.collisions = me->collisions + 1;


              newMe.targetPos = newTarget.second;
              newMe.newDir = newTarget.first;
              newMe.finalTargetPos = me->finalTargetPos;
              newMe.careAboutFieldEdge = me->careAboutFieldEdge;
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
                if (lhs->collisions > rhs->collisions) return true;
                else if (lhs->collisions < rhs->collisions) return false;
                else
                    return abs((lhs->pos - lhs->finalTargetPos).length())
                            > abs((rhs->pos - rhs->finalTargetPos).length());
            }
          };

        };

        std::priority_queue<NumRobotPtr, std::vector<NumRobotPtr>, NumRobot::CustomCompare> robotQueue;
        NumRobot me;

        std::vector<Vector2> displayData;
        double errorMargin = constants::GOTOPOS_LUTH_ERROR_MARGIN;

        Vector2 targetPos = {999.2, 999.2};
        ros::Time startTime;

        Controller velPID;
        Controller posPID;
        bool pidInit = false;
        bool avoidBall = false;
        bool canGoOutsideField = true;
        bool tracePath(NumRobot &numRobot, Vector2 target);
        bool calculateNumericDirection(RobotPtr robot, NumRobot &me);
        void drawCross(Vector2 &pos);
        bool calculateNextPoint(NumRobotPtr me);
    public:
        void clear();
        Vector2 goToPos(RobotPtr robot, Vector2 &target);
        void setAvoidBall(bool _avoidBall);
        void setCanGoOutsideField(bool _canGoOutsideField);
};

} // control
} // ai
} // rtt

#endif //ROBOTEAM_AI_CONTROLGOTOPOSLUTH_H