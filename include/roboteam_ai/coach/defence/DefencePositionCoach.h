//
// Created by rolf on 18-2-19.
//

#ifndef ROBOTEAM_AI_DEFENSIVECOACH_H
#define ROBOTEAM_AI_DEFENSIVECOACH_H

#include <map>
#include <set>

#include <gtest/gtest_prod.h>
#include <roboteam_utils/Vector2.h>
#include "world/WorldData.h"
#include "world/World.h"
#include "PossiblePass.h"

namespace rtt::ai::coach {

    using Line = std::pair<Vector2, Vector2>;
    enum botType {
        BLOCKBALL, BLOCKTOGOAL, BLOCKPASS, BLOCKONLINE
    };

    struct DefenderBot {
        int id;
        Vector2 targetPos;
        double orientation;
        int blockFromID;
        botType type;

        int coveredCount = 0;

        [[nodiscard]] world::Robot::RobotPtr toRobot() const noexcept;

        [[nodiscard]] bool validPosition(const world::WorldData &world) const noexcept;
    };

    class DefencePositionCoach {
        FRIEND_TEST(defensive_coach, blockPoints);

    public:
        std::map<int, DefenderBot> decidePositions(const std::map<int, DefenderBot> &lockedDefenders,
                                                   const std::set<int> &freeRobots) noexcept;

        std::tuple<bool, int, std::set<int>> decideLockedPositions(const std::map<int, DefenderBot> &lockedDefenders,
                                                                   std::set<int> freeRobots) noexcept;

        constexpr static double CALCULATION_COLLISION_RADIUS = 0.15;
        constexpr static double DEFENSELINE_MARGIN = 0.15;
        constexpr static double SEARCH_POINTS = 31.0;
    private:
        world::WorldData simulatedWorld;
        std::map<int, DefenderBot> defenders;

        [[nodiscard]] std::optional<DefenderBot> blockMostDangerousPos() const noexcept;

        [[nodiscard]] std::optional<DefenderBot> blockPass(const PossiblePass &pass) const noexcept;

        void addDefender(const DefenderBot &defender) noexcept;

        void assignIDs(int lockedCount, std::set<int> freeRobotIDs, const std::map<int, DefenderBot> &oldDefenders) noexcept;

        void mostDangerousBlocking(const DefenderBot &lockedDefender, bool &blockedMostDangerousPos, int &lockedCount, bool &replacedDefender);

        void setNewDefender(const DefenderBot &lockedDefender, const PossiblePass &pass, int &lockedCount, bool &replacedDefender);
    };

    namespace defender_pos {
        [[nodiscard]] Vector2 getMostDangerousPos(const world::WorldData &world);

        [[nodiscard]] std::vector<PossiblePass>
        sortPassesByDanger(std::vector<std::pair<PossiblePass, double>> &passesWithDanger);

        [[nodiscard]] std::vector<std::pair<PossiblePass, double>> createPassesAndDanger(const world::WorldData &world);

        void removeBotFromWorld(world::WorldData &world, int id, bool ourTeam);

        [[nodiscard]] world::WorldData getTheirAttackers(const world::WorldData &world);

        [[nodiscard]] rtt::ai::coach::DefenderBot createBlockPass(PossiblePass const &pass, const Vector2 &blockPoint);

        [[nodiscard]] world::WorldData setupSimulatedWorld();

        [[nodiscard]] Vector2 getBlockPoint(const Line &openGoalSegment, const Vector2 &point, double collisionRadius);

        [[nodiscard]] Line shortenLineForDefenseArea(const Vector2 &lineStart, const Vector2 &lineEnd, double defenseMargin);

        [[nodiscard]] bool validNewPosition(const Vector2 &position, const world::WorldData &world);

        [[nodiscard]] std::optional<double> pickNewPosition(const Line &line, const world::WorldData &world);

        [[nodiscard]] double maxX();//furthest point forwards the availableIDs can go

        [[nodiscard]] std::optional<Vector2> pickNewPosition(PossiblePass pass, const world::WorldData &world);

        [[nodiscard]] Vector2 getPosOnLine(const Line &line, double aggressionFactor);


        [[nodiscard]] std::optional<Line> getBlockLineSegment(const Line &openGoalSegment, const Vector2 &point,
                                                              double collisionRadius = Constants::ROBOT_RADIUS() +
                                                                                       Constants::BALL_RADIUS(),
                                                              double margin = -1.0);

        [[nodiscard]] std::optional<Vector2> blockOnDefenseLine(const Line &openGoalSegment, const Vector2 &point);

        [[nodiscard]] double getOrientation(const Line &line);

        [[nodiscard]] std::vector<PossiblePass> createPassesSortedByDanger(const world::WorldData &world);

        [[nodiscard]] DefenderBot createBlockBall(const Line &blockLine);

        [[nodiscard]] DefenderBot
        createBlockToGoal(const PossiblePass &pass, double aggressionFactor, const Line &blockLine);

        [[nodiscard]] DefenderBot
        createBlockToGoal(const PossiblePass &pass, const Vector2 &position, const Line &blockLine);

        [[nodiscard]] DefenderBot createBlockOnLine(const PossiblePass &pass, const Vector2 &blockPos);

        [[nodiscard]] std::optional<Line>
        blockToGoalLine(const PossiblePass &pass, const world::WorldData &simulatedWorld);

        [[nodiscard]] std::optional<Line> blockBallLine(const world::WorldData &simulatedWorld);

        [[nodiscard]] std::optional<Vector2> blockOnDefenseAreaLine(const PossiblePass &pass,
                                                                    const world::WorldData &simulatedWorld);

        [[nodiscard]] Vector2 findPositionForBlockBall(const Line &line);
    } // defender

    extern DefencePositionCoach g_defensivePositionCoach;


} // rtt::ai::coach

#endif //ROBOTEAM_AI_DEFENSIVECOACH_H
