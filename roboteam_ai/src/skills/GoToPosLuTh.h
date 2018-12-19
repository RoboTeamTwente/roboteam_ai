//
// Created by thijs on 04-12-18.
//
#include "Skill.h"
#include <queue>
#include <random>  //random numbers..
#include <cstdlib>
#include <time.h>
#include <cmath>

#include "../interface/drawer.h"
#include <roboteam_ai/src/skills/DefendOnRobot.h>
// #include <roboteam_ai/src/utilities/Coach.h>

// #include "../interface/Interface.h"

#ifndef ROBOTEAM_AI_GOTOPOSLUTH_H
#define ROBOTEAM_AI_GOTOPOSLUTH_H

namespace rtt {
namespace ai {
class GoToPosLuTh : public Skill {

    private:
        struct NumRobot;
        using NumRobotPtr = std::shared_ptr<NumRobot>;
        struct NumRobot {

          int id = 0;                       //Robot id
          unsigned long startIndex = 0;
          Vector2 pos;                  //Current x,y position in m
          Vector2 targetPos;            //Target position in m
          Vector2 finalTargetPos;
          Vector2 vel;                  //Current x,y velocity in ms-1
          Vector2 targetVel;            //Target velocity in ms-1
          double maxVel = 2.5;          //Maximum velocity in ms-1
          Vector2 acc;                  //Current x,y acceleration in ms-2
          double maxAcc = 2.5;          //Maximum acceleration in ms-2
          std::vector<Vector2> posData; //Save the position data
          std::vector<Vector2> velData; //Save the velocity data
          float t = 0;
          const float dt = 0.05f;
          int totalCalculations = 0;
          int collisions = 0;

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

          std::vector<Vector2> getNewTargets(Vector2 &collisionPos, Vector2 &startPos) {
              std::vector<Vector2> newTargets;

              Vector2 deltaPos = collisionPos - startPos;

              std::vector<double> angles = {- M_PI*0.0625, M_PI*0.0625};
              for (double angle : angles) {
                  Vector2 newTarget = startPos + deltaPos.rotate(angle);
                  newTargets.push_back(newTarget);
              }
              return newTargets;
          }

          NumRobotPtr getNewNumRobot(const NumRobotPtr &me, Vector2 &newTarget) {
              NumRobot newMe;

              std::vector<Vector2> _posData(me->posData.begin(), me->posData.begin() + me->startIndex + 1);
              newMe.posData = _posData;
              newMe.pos = newMe.posData.back();//me->posData[me->startIndex];
              std::vector<Vector2> _velData(me->velData.begin(), me->velData.begin() + me->startIndex + 1);
              newMe.velData = _velData;
              newMe.vel = newMe.velData.back();//me->velData[me->startIndex];

              newMe.id = me->id;
              newMe.totalCalculations = me->totalCalculations;
              newMe.collisions = me->collisions + 1;
              newMe.startIndex = newMe.posData.size();
              newMe.targetPos = newTarget;
              newMe.finalTargetPos = targetPos;

              return std::make_shared<NumRobot>(newMe);

          }

          struct CustomCompare {
            bool operator()(const NumRobotPtr &lhs, const NumRobotPtr &rhs) {
                if (lhs->collisions < rhs->collisions) return false;
                else
                    return abs((lhs->pos - lhs->finalTargetPos).length())
                            > abs((rhs->pos - rhs->finalTargetPos).length());
            }
          };

        };

        std::priority_queue<NumRobotPtr, std::vector<NumRobotPtr>, NumRobot::CustomCompare> robotQueue;

        bool tracePath(NumRobot &numRobot, Vector2 target);
        std::vector<Vector2> displayData;
        double errorMargin = 0.3;

        bool drawInterface;
        bool goToBall;
        bool passiveDefend;
        bool random;

        enum Progression {
          ON_THE_WAY, DONE, FAIL
        };
        Progression currentProgress;
        Progression checkProgression();

        Vector2 targetPos;

        bool checkTargetPos(Vector2 pos);
        void sendMoveCommand();
        bool calculateNumericDirection(NumRobot &me, roboteam_msgs::RobotCommand &command);
        void drawCross(Vector2 &pos);
        bool calculateNextPoint(NumRobotPtr me);

    public:
        explicit GoToPosLuTh(string name, bt::Blackboard::Ptr blackboard);
        void onInitialize() override;
        Status onUpdate() override;
        void onTerminate(Status s) override;
};
} // ai
} // rtt


#endif //ROBOTEAM_AI_GOTOPOSLUTH_H
