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
    public:
        enum botType { BLOCKBALL, BLOCKTOGOAL, WALL, BLOCKPASS, BLOCKONLINE };
        struct DefenderBot {
          int id;
          Vector2 targetPos;
          double orientation;
          int blockFromID;
          botType type;
          world::Robot toRobot();
          bool validPosition(const world::WorldData &world);
        };
        struct BlockPassBot : DefenderBot {
          int blockToID;
        };
        Vector2 getMostDangerousPos(const world::WorldData &world);
        std::vector<DefenderBot> decidePositions(int amount);

        std::shared_ptr<DefenderBot> createBlockBall(const world::WorldData &simulatedWorld);
        std::shared_ptr<DefenderBot> tryBlockToGoal(const PossiblePass &pass, double aggressionFactor,
                const world::WorldData &simulatedWorld);
        std::shared_ptr<BlockPassBot> tryBlockPass(PossiblePass &pass, const world::WorldData &simulatedWorld);
        std::shared_ptr<DefenderBot> tryBlockOnLine(const PossiblePass &pass, const world::WorldData &simulatedWorld);

        std::shared_ptr<Line> getBlockLineSegment(const Line &openGoalSegment, const Vector2 &point,
                double collisionRadius = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS(),
                double margin = - 1.0);
        std::shared_ptr<Vector2> blockOnDefenseLine(const Line &openGoalSegment, const Vector2 &point);
        Vector2 getBlockPoint(const Line &openGoalSegment, const Vector2 &point, double collisionRadius);
        Line shortenLineForDefenseArea(const Vector2 &lineStart, const Vector2 &lineEnd, double defenseMargin);
        Vector2 getPosOnLine(const Line &line, double aggressionFactor);
        double getOrientation(const Line &line);
        Vector2 findPositionForBlockBall(const Line &line);
    private:
        std::vector<PossiblePass> createPassesSortedByDanger(const world::WorldData &world);
        std::vector<PossiblePass> sortPassesByDanger(std::vector<std::pair<PossiblePass, double>> &passesWithDanger);
        std::vector<std::pair<PossiblePass, double>> createPassesAndDanger(const world::WorldData &world);
        world::WorldData removeBotFromWorld(world::WorldData world, int id, bool ourTeam);

};
extern DefencePositionCoach g_defensivePositionCoach;

}
}
}

#endif //ROBOTEAM_AI_DEFENSIVECOACH_H
