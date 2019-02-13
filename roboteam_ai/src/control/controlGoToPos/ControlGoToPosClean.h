//
// Created by rolf on 5-2-19.
//

#ifndef ROBOTEAM_AI_CONTROLGOTOPOSCLEAN_H
#define ROBOTEAM_AI_CONTROLGOTOPOSCLEAN_H

#include "GoToPosInclude.h"
#include <roboteam_ai/src/interface/InterfaceValues.h>
#include <chrono>

namespace rtt {
namespace ai {
namespace control {

class ControlGoToPosClean {
    private:
        //constants, should be moved at some point, or adapted in a dynamic model (e.g. for lower speeds for certain branches, jazz like that)
        double dt = 0.03;
        double maxVel = 1.56;
        double maxAcc = 3.03;

        double defaultRobotCollisionRadius = 3*constants::ROBOT_RADIUS_MAX;
        int robotID = - 1;
        Vector2 pos;
        Vector2 vel;
        Controller velPID;
        Controller posPID;
        bool pidInit = false;
        bool avoidBall = false;
        bool canGoOutsideField = true;
        bool pidInitialized = false;

        void drawCross(Vector2 &pos, QColor color = Qt::green);
        void drawPoint(Vector2 &pos, QColor color = Qt::green);

            std::vector<std::pair<Vector2, QColor>> displayData;
        void initializePID();
        void checkInterfacePID();
        Vector2 computeCommand(std::shared_ptr<roboteam_msgs::WorldRobot> robot);
        bool doRecalculatePath(std::shared_ptr<roboteam_msgs::WorldRobot> robot, Vector2 targetPos);
        double remainingStraightLinePathLength(Vector2 currentPos, Vector2 halfwayPos, Vector2 finalPos);
        void drawInInterface();
        std::chrono::duration<double> time;

        // If there is another way to return a shared pointer from an object to itself that is more pretty let me know
        struct PathPoint : std::enable_shared_from_this<PathPoint> {
            public:
                Vector2 currentTarget;//Either the endPoint or an in between target
                Vector2 pos;
                Vector2 vel;
                Vector2 acc;
                double t;
                int collisions;
                bool hasBeenTicked;
                std::shared_ptr<PathPoint> parent;
                std::vector<std::shared_ptr<PathPoint>> children;
                std::shared_ptr<PathPoint> backTrack(double backTime);
                std::shared_ptr<PathPoint> backTrack(int maxCollisionDiff);
                std::shared_ptr<PathPoint> backTrack(double backTime, int maxCollisionDiff);

                void addChild(std::shared_ptr<PathPoint> &newChild);
                void addChildren(std::vector<std::shared_ptr<PathPoint>> &newChildren);
                bool isCollision(Vector2 target, double distance);
        };

        std::pair<std::vector<Vector2>, std::shared_ptr<PathPoint>> getNewTargets(
                std::shared_ptr<PathPoint> collisionPoint);
        bool checkCollision(std::shared_ptr<PathPoint> point);
        std::shared_ptr<PathPoint> computeNewPoint(std::shared_ptr<PathPoint> oldPoint, Vector2 subTarget);
        void tracePath(std::shared_ptr<roboteam_msgs::WorldRobot> robot);
        std::vector<PathPoint> backTrackPath(std::shared_ptr<PathPoint> endPoint, std::shared_ptr<PathPoint> root);
        Vector2 findCollisionPos(std::shared_ptr<PathPoint> point);

        bool branchHasTarget(std::shared_ptr<PathPoint> newBranchStart, Vector2 target) {
            for (const auto &child : newBranchStart->children) {
                if (child->currentTarget == target) {
                    return true;
                }
            }
            return false;
        }

        std::vector<PathPoint> path;
    public:
        Vector2 finalTargetPos;
        Vector2 goToPos(std::shared_ptr<roboteam_msgs::WorldRobot> robot, Vector2 targetPos);
        void setAvoidBall(bool _avoidBall);
        void setCanGoOutsideField(bool _canGoOutsideField);

};

}
}
}

#endif //ROBOTEAM_AI_CONTROLGOTOPOSCLEAN_H
