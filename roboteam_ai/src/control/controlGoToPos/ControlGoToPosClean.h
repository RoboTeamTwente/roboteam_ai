//
// Created by rolf on 5-2-19.
//

#ifndef ROBOTEAM_AI_CONTROLGOTOPOSCLEAN_H
#define ROBOTEAM_AI_CONTROLGOTOPOSCLEAN_H
#include "GoToPosInclude.h"
#include <roboteam_ai/src/interface/InterfaceValues.h>

namespace rtt{
namespace ai{
namespace control{
class ControlGoToPosClean {
    private:
        int robotID;
        Vector2 pos;
        Vector2 vel;
        Vector2 finalTargetPos;
        Controller velPID;
        Controller posPID;
        bool avoidBall=false;
        bool canGoOutsideField=true;
        bool pidInitialized=false;
        void drawCross(Vector2 &pos);
        std::vector<Vector2> displayData;
        void initializePID();
        void checkInterfacePID();
        Vector2 sendCommand();
        void drawInInterface();

        struct PathPoint :std::enable_shared_from_this<PathPoint> {
          Vector2 currentTarget;//Either the endPoint or an in between target
          Vector2 pos;
          Vector2 vel;
          Vector2 acc;
          double t;
          std::shared_ptr<PathPoint> middle,left,right,parent;
          std::shared_ptr<PathPoint> backTrack(double toTime);
          void addChild(std::shared_ptr<PathPoint> middleChild);
          void addChildren(std::shared_ptr<PathPoint> leftChild, std::shared_ptr<PathPoint> rightChild);

          bool atTarget(Vector2 target);
        };
        bool checkCollission(PathPoint point);
        PathPoint computeNewPoint(PathPoint oldPoint, Vector2 subTarget);

    public:
        Vector2 goToPos(std::shared_ptr<roboteam_msgs::WorldRobot> robot,Vector2 targetPos);
        void setAvoidBall(bool _avoidBall);
        void setCanGoOutsideField(bool _canGoOutsideField);

};
}
}
}


#endif //ROBOTEAM_AI_CONTROLGOTOPOSCLEAN_H
