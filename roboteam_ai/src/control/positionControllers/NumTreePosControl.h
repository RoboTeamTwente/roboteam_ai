//
// Created by rolf on 5-2-19.
//

#ifndef ROBOTEAM_AI_CONTROLGOTOPOSCLEAN_H
#define ROBOTEAM_AI_CONTROLGOTOPOSCLEAN_H

#include "PositionControlIncludes.h"
#include <roboteam_ai/src/interface/InterfaceValues.h>

namespace rtt {
namespace ai {
namespace control {

class NumTreePosControl {
    private:
        //constants, should be moved at some point, or adapted in a dynamic model (e.g. for lower speeds for certain branches, jazz like that)
        const double dt = 0.075;
        const double defaultRobotCollisionRadius = 3*Constants::ROBOT_RADIUS_MAX();
        int robotID = - 1;
        Vector2 pos;
        Vector2 vel;
        bool avoidBall = false;
        bool canGoOutsideField = true;

        void drawCross(Vector2 &pos, QColor color = Qt::green);
        void drawPoint(Vector2 &pos, QColor color = Qt::green);

            std::vector<std::pair<Vector2, QColor>> displayData;
        bool doRecalculatePath(std::shared_ptr<roboteam_msgs::WorldRobot> robot, Vector2 targetPos);
        double remainingStraightLinePathLength(Vector2 currentPos, Vector2 halfwayPos, Vector2 finalPos);
        void drawInInterface();

        // If there is another way to return a shared pointer from an object to itself that is more pretty let me know
        struct PathPoint : std::enable_shared_from_this<PathPoint> {
            private:
                double maxV = 3.4;
                double maxA = 5.1;
            public:
                Vector2 currentTarget;  //Either the endPoint or an in between target
                Vector2 finalTarget;    //Always the endPoint
                Vector2 pos;
                Vector2 vel;
                Vector2 acc;
                double maxVel() {
                    double distanceRemaining = (finalTarget-pos).length();
                    double absoluteMax = sqrt(2.0*maxAcc()*distanceRemaining);
                    return absoluteMax > maxV ? maxV : absoluteMax;
                }
                double maxAcc() {
                    return vel.length() > 0.5 * maxV ? maxA * 0.5 : maxA * ((maxV - vel.length()) / maxV);
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

        enum GTPType {
          numeric,
          force
        };
        PosVelAngle computeNumericCommand(std::shared_ptr<roboteam_msgs::WorldRobot> robot);
        PosVelAngle computeForceCommand(std::shared_ptr<roboteam_msgs::WorldRobot> robot);
        PosVelAngle computeCommand(std::shared_ptr<roboteam_msgs::WorldRobot> robot, GTPType gtpType = numeric);


        std::pair<std::vector<Vector2>, std::shared_ptr<PathPoint>> getNewTargets(
                std::shared_ptr<PathPoint> collisionPoint);
        bool checkCollision(std::shared_ptr<PathPoint> point, double collisionRadius = 0.27);
        std::shared_ptr<PathPoint> computeNewPoint(std::shared_ptr<PathPoint> oldPoint, Vector2 subTarget);
        void tracePath(std::shared_ptr<roboteam_msgs::WorldRobot> robot);
        std::vector<PathPoint> backTrackPath(std::shared_ptr<PathPoint> endPoint, std::shared_ptr<PathPoint> root);
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

#endif //ROBOTEAM_AI_CONTROLGOTOPOSCLEAN_H
