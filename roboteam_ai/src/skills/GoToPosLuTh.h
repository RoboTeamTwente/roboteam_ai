//
// Created by thijs on 04-12-18.
//
#include "Skill.h"
#include <cmath>
#include <queue>
#include <random>  //random numbers..
#include <cstdlib>
#include <time.h>
#include "../interface/drawer.h"

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

          int id;                       //Robot id
          unsigned long startIndex = 0;
          Vector2 pos;                  //Current x,y position in m
          Vector2 targetPos;            //Target position in m
          Vector2 vel;                  //Current x,y velocity in ms-1
          Vector2 targetVel;            //Target velocity in ms-1
          double maxVel = 1.5;          //Maximum velocity in ms-1
          Vector2 acc;                  //Current x,y acceleration in ms-2
          double maxAcc = 2.5;          //Maximum acceleration in ms-2
          std::vector<Vector2> posData; //Save the position data
          std::vector<Vector2> velData; //Save the velocity data
          float t = 0;
          const float dt = 0.05;
          int totalCalculations = 0;

          Vector2 getDirection() {
              return (targetPos - pos).normalize();
          }

          Vector2 getDirection(Vector2 &target) {
              return (target - pos).normalize();
          }

          bool isCollision(Vector2 &otherPos) {
              double minDistance = 0.2;
              return isCollision(otherPos, minDistance);
          }

          bool isCollision(Vector2 &otherPos, double minDistance) {
              return (std::abs((otherPos - pos).length()) < minDistance);
          }

          struct CustomCompare {
            bool operator()(NumRobotPtr lhs, NumRobotPtr rhs)
            {
                return lhs->posData.size() < rhs->posData.size();
            }
          };

        };

        std::priority_queue< NumRobotPtr, std::vector<NumRobotPtr>, NumRobot::CustomCompare > robotQueue;

        bool tracePath(NumRobot &numRobot, Vector2 target);

        std::vector<Vector2> displayData;
        using Status = bt::Node::Status;
        roboteam_msgs::WorldRobot robot;

        bool drawInterface;
        bool goToBall;
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

    public:
        explicit GoToPosLuTh(string name, bt::Blackboard::Ptr blackboard);
        std::string node_name() override;
        void initialize() override;
        Status update() override;
        void terminate(Status s) override;
        bool calculateNextPoint(NumRobotPtr me, Vector2 &target);
};
} // ai
} // rtt


#endif //ROBOTEAM_AI_GOTOPOSLUTH_H
