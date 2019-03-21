//
// Created by rolf on 5-2-19.
//

#ifndef ROBOTEAM_AI_NUMTREEPOSCONTROL_H
#define ROBOTEAM_AI_NUMTREEPOSCONTROL_H

#include "PositionControlIncludes.h"
#include <roboteam_ai/src/interface/InterfaceValues.h>

namespace rtt {
namespace ai {
namespace control {

class NumTreePosControl {
    private:
        using InterfaceValues = interface::InterfaceValues;

        const double dt = 0.07;
        const double defaultRobotCollisionRadius = 3*Constants::ROBOT_RADIUS_MAX();
        int robotID = - 1;
        Vector2 pos;
        Vector2 vel;
        bool avoidBall = false;
        bool canGoOutsideField = true;

        void drawCross(Vector2 &pos, QColor color = Qt::green);
        void drawPoint(Vector2 &pos, QColor color = Qt::green);
        void addDataInInterface(std::vector<std::pair<rtt::Vector2, QColor>> displayColorData);
        void redrawInInterface();

        std::vector<std::pair<Vector2, QColor>> displayData;
        bool doRecalculatePath(std::shared_ptr<roboteam_msgs::WorldRobot> robot, Vector2 targetPos);
        double remainingStraightLinePathLength(Vector2 currentPos, Vector2 halfwayPos, Vector2 finalPos);

        // If there is another way to return a shared pointer from an object to itself that is more pretty let me know
        struct PathPoint : std::enable_shared_from_this<PathPoint> {
            private:
                double maxV = 2.0;
                double maxAccAtLowV = 6.1;
                double maxAccAtHighV = 3.1;
                double maxDecelleration = 6.1;
            public:
                Vector2 currentTarget;  //Either the endPoint or an in between target
                Vector2 finalTarget;    //Always the endPoint
                Vector2 pos;
                Vector2 vel;
                Vector2 acc;

                double maxVel() {
                    double distanceRemaining = (finalTarget - pos).length();
                    double absoluteMax = sqrt(2.0*maxAcc()*distanceRemaining)*0.8;
                    return absoluteMax > maxV ? maxV : absoluteMax;
                }
                double maxAcc() {
                    return vel.length() > maxV*0.5 ?
                           maxAccAtHighV :
                           maxAccAtLowV - (maxAccAtLowV - maxAccAtHighV)*((maxV - vel.length())/maxV);
                }
                double maxDec() {
                    return maxDecelleration;
                }

                double t;
                int collisions;
                std::shared_ptr<PathPoint> parent;
                std::vector<std::shared_ptr<PathPoint>> children;
                std::shared_ptr<PathPoint> backTrack(double backTime);
                std::shared_ptr<PathPoint> backTrack(int maxCollisionDiff);
                std::shared_ptr<PathPoint> backTrack(double backTime, int maxCollisionDiff);

                void addChild(std::shared_ptr<PathPoint> &newChild);
                void addChildren(std::vector<std::shared_ptr<PathPoint>> &newChildren);
                bool isCollision(Vector2 target, double distance);

                bool branchHasTarget(const Vector2 &target);
                bool anyBranchHasTarget(const Vector2 &target);
                bool anyChildHasTarget(const Vector2 &target);
                bool anyParentHasTarget(const Vector2 &target);

        };

        PosVelAngle computeCommand(std::shared_ptr<roboteam_msgs::WorldRobot> robot);

        std::pair<std::vector<Vector2>, std::shared_ptr<PathPoint>> getNewTargets(
                std::shared_ptr<PathPoint> collisionPoint);
        bool checkCollision(std::shared_ptr<PathPoint> point, double collisionRadius = 0.27);
        std::shared_ptr<PathPoint> computeNewPoint(std::shared_ptr<PathPoint> oldPoint, Vector2 subTarget);
        void tracePath(std::shared_ptr<roboteam_msgs::WorldRobot> robot);
        std::vector<PathPoint> backTrackPath(std::shared_ptr<PathPoint> point, std::shared_ptr<PathPoint> root);
        Vector2 findCollisionPos(std::shared_ptr<PathPoint> point, double collisionRadius = 0.27);

        std::vector<PathPoint> path;
    public:
        void clear();
        Vector2 finalTargetPos;
        PosVelAngle goToPos(std::shared_ptr<roboteam_msgs::WorldRobot> robot, Vector2 targetPos);
        void setAvoidBall(bool _avoidBall);
        void setCanGoOutsideField(bool _canGoOutsideField);

};

}
}
}

#endif //ROBOTEAM_AI_NUMTREEPOSCONTROL_H
