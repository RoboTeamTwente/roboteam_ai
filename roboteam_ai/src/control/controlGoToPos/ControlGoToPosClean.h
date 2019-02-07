//
// Created by rolf on 5-2-19.
//

#ifndef ROBOTEAM_AI_CONTROLGOTOPOSCLEAN_H
#define ROBOTEAM_AI_CONTROLGOTOPOSCLEAN_H
#include "GoToPosInclude.h"
#include <roboteam_ai/src/interface/InterfaceValues.h>

namespace rtt {
namespace ai {
namespace control {
class ControlGoToPosClean {
    private:
        //constants, should be moved at some point, or adapted in a dynamic model (e.g. for lower speeds for certain branches, jazz like that)
        double dt = 0.0175;
        double maxVel = 1.56;
        double maxAcc = 3.03;

        int robotID = - 1;
        Vector2 pos;
        Vector2 vel;
        Controller velPID;
        Controller posPID;
        bool avoidBall = false;
        bool canGoOutsideField = true;
        bool pidInitialized = false;
        void drawCross(Vector2 &pos);
        std::vector<Vector2> displayData;
        void initializePID();
        void checkInterfacePID();
        Vector2 computeCommand();
        void drawInInterface();


        // If there is another way to return a shared pointer from an object to itself that is more pretty let me know
        struct PathPoint : std::enable_shared_from_this<PathPoint> {
          Vector2 currentTarget;//Either the endPoint or an in between target
          Vector2 pos;
          Vector2 vel;
          Vector2 acc;
          double t;
          std::shared_ptr<PathPoint> middle, left, right, parent;
          std::shared_ptr<PathPoint> backTrack(double toTime);
          void addChild(std::shared_ptr<PathPoint> middleChild);
          void addChildren(std::shared_ptr<PathPoint> leftChild, std::shared_ptr<PathPoint> rightChild);
          bool isCollission(Vector2 target, double distance);
        };
        class CustomCompare {
            public:
                explicit CustomCompare(ControlGoToPosClean& x) : parent(x) {};
                bool operator()(std::shared_ptr<PathPoint> lhs, std::shared_ptr<PathPoint> rhs);
            private:
                ControlGoToPosClean& parent;
        };

        std::pair<Vector2, Vector2> getNewTargets(std::shared_ptr<PathPoint> collisionPoint, Vector2 startPos);
        std::priority_queue<std::shared_ptr<PathPoint>,
                            std::vector<std::shared_ptr<PathPoint>>> pathQueue;
        bool checkCollission(std::shared_ptr<PathPoint> point);
        std::shared_ptr<PathPoint> computeNewPoint(std::shared_ptr<PathPoint> oldPoint, Vector2 subTarget);
        std::vector<PathPoint> tracePath(std::shared_ptr<PathPoint> root);
        std::vector<PathPoint> backTrackPath(std::shared_ptr<PathPoint> endPoint, std::shared_ptr<PathPoint> root);
        Vector2 findCollisionPos(std::shared_ptr<PathPoint> point);
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
