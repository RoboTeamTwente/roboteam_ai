//
// Created by rolf on 18-2-19.
//

#ifndef ROBOTEAM_AI_DEFENSIVECOACH_H
#define ROBOTEAM_AI_DEFENSIVECOACH_H
#include "roboteam_utils/Vector2.h"
#include "roboteam_ai/src/world/World.h"
#include "PossiblePass.h"
namespace rtt {
namespace ai {
namespace coach {
using Line=std::pair<Vector2, Vector2>;

class DefencePositionCoach {
    private:
        std::vector<roboteam_msgs::WorldRobot> createVirtualBots(const std::vector<Vector2>& decidedBlocks);

    public:
        enum botType { BLOCKBALLTOGOAL, BLOCKTOGOAL, WALL, BLOCKPASS, BLOCKONLINE };
        struct DefenderBot {
          int id;
          Vector2 targetPos;
          double orientation;
          int blockFromID;
          botType type;
        };
        struct BlockPassBot : DefenderBot {
          int blockToID;
        };

        std::vector<Vector2> doubleBlockOnDefenseLine(const std::pair<Vector2, Vector2>& openGoalSegment, const Vector2& point);
        std::vector<std::pair<Vector2, double>> decideDefendersOnDefenseLine(int amount);
        std::vector<DefenderBot> decidePositions(int amount);

        std::shared_ptr<DefenderBot> createBlockToGoal(const PossiblePass& pass, double aggressionFactor,const roboteam_msgs::World &simulatedWorld);
        std::shared_ptr<BlockPassBot> createBlockPass(const PossiblePass& pass, const roboteam_msgs::World &simulatedWorld);
        std::shared_ptr<DefenderBot> createBlockOnLine(const PossiblePass& pass,const roboteam_msgs::World &simulatedWorld);

        std::shared_ptr<Line> getBlockLineSegment(const Line& openGoalSegment, const Vector2& point, double collisionRadius=Constants::ROBOT_RADIUS()+Constants::BALL_RADIUS(),
                double margin = - 1.0);
        std::shared_ptr<Vector2> blockOnDefenseLine(const Line& openGoalSegment, const Vector2& point);
        Vector2 getBlockPoint(const Line& openGoalSegment, const Vector2& point, double collisionRadius);
        Line shortenLineForDefenseArea(const Vector2& lineStart, const Vector2& lineEnd, double defenseMargin);
        Vector2 getPosOnLine(const Line& line, double aggressionFactor);
        double getOrientation(const Line& line);
    private:
        std::vector<PossiblePass> createPassesSortedByDanger(const roboteam_msgs::World &world);
};
extern DefencePositionCoach g_defensivePositionCoach;

}
}
}

#endif //ROBOTEAM_AI_DEFENSIVECOACH_H
