//
// Created by thijs on 19-11-18.
//

#include "Skill.h"
#include <cmath>

#ifndef ROBOTEAM_AI_GOTOPOSLUTH_OLD_H
#define ROBOTEAM_AI_GOTOPOSLUTH_OLD_H

namespace rtt {
namespace ai {
class GoToPosLuTh_OLD : public Skill {

    private:

        struct numRobot {
          int id;                       //Robot id
          double angle;                 //Current angle
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
              if (vel.length() > maxVel*0.5) {
                  minDistance *= 1.5;
              }
              return (std::abs((otherPos - pos).length()) < minDistance);
          }
        };
        bool tracePath(numRobot &me, int &startIndex, Vector2 target, bool semiPath);
        bool avoidObject(numRobot &me, int &startIndex, bool firstTry);

        std::vector<Vector2> displayData;
        using Status = bt::Node::Status;

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
        bool calculateNumericDirection(numRobot &me, roboteam_msgs::RobotCommand &command);
        Vector2 getClosestRobotPos(const roboteam_msgs::World &world, numRobot &me);
    public:
        explicit GoToPosLuTh_OLD(string name, bt::Blackboard::Ptr blackboard);
        void onInitialize() override;
        Status onUpdate() override;
        void onTerminate(Status s) override;
};

} // ai
} // rtt


#endif //ROBOTEAM_AI_GOTOPOSLUTH_H
