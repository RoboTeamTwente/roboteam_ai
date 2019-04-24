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
        Vector2 getMostDangerousPos(const world::WorldData &world);
        std::vector<DefenderBot> decidePositions(int amount);

        DefenderBot createBlockBall(const Line &blockLine);
        DefenderBot createBlockToGoal(const PossiblePass &pass, double aggressionFactor, const Line &blockLine);
        DefenderBot createBlockPass(PossiblePass &pass, const Vector2 &blockPoint);
        DefenderBot createBlockOnLine(const PossiblePass &pass, const Vector2 &blockPos);

        std::shared_ptr<Line> blockToGoalLine(const PossiblePass &pass, const world::WorldData &simulatedWorld);
        std::shared_ptr<Line> blockBallLine(const world::WorldData &simulatedWorld);
        std::shared_ptr<Vector2> blockOnPassLine(PossiblePass &pass, const world::WorldData &simulatedWorld);
        std::shared_ptr<Vector2> blockOnDefenseAreaLine(const PossiblePass &pass,
                const world::WorldData &simulatedWorld);
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
        const double defenceLineMargin=0.15; //min distance the points are from defence area. Should atleast be robotradius large.
        const double calculationCollisionRad=0.15; // distance at which our own robots are considered to be colliding in our calculation (prevents robots from stacking up too much)

        world::WorldData simulatedWorld;
        std::vector<DefenderBot> defenders;

        std::vector<PossiblePass> createPassesSortedByDanger(const world::WorldData &world);
        std::vector<PossiblePass> sortPassesByDanger(std::vector<std::pair<PossiblePass, double>> &passesWithDanger);
        std::vector<std::pair<PossiblePass, double>> createPassesAndDanger(const world::WorldData &world);
        world::WorldData removeBotFromWorld(world::WorldData world, int id, bool ourTeam);
        world::WorldData getTheirAttackers(const world::WorldData& world);

        bool validNewPosition(const Vector2& position,const world::WorldData& world);
        std::shared_ptr<double> pickNewPosition(const Line& line ,const world::WorldData& world);
        std::shared_ptr<Vector2> pickNewPosition(PossiblePass pass, const world::WorldData& world);

        world::WorldData setupSimulatedWorld();
        void blockMostDangerousPos();
        bool blockPass(PossiblePass pass);
        void addDefender(DefenderBot defender);


};
extern DefencePositionCoach g_defensivePositionCoach;

}
}
}

#endif //ROBOTEAM_AI_DEFENSIVECOACH_H
