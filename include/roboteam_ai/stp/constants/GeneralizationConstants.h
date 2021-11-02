//
// Created by maxl on 19-03-21.
//

#ifndef RTT_GENERALIZATIONCONSTANTS_H
#define RTT_GENERALIZATIONCONSTANTS_H

#include <optional>
#include <roboteam_utils/Grid.h>

namespace rtt::ai::stp::gen {
    /**
     * Struct that is used to store computations made with this module.
     * Save computations here that are usable for each robot for a position.
     *       DO NOT save values specific to a robot in here (like TimeToPosition).
     * If a position's score for a specific evaluation already had been computed in the tick, it will
     * use that value instead of recomputing it. If it was not computed yet, it will compute and save it.
     *
     * @memberof scoreOpen : uint8_t score for the Openness of a position -> evaluations/position/OpennessEvaluation
     * @memberof scoreLineOfSight : uint8_t score for the LineOfSight to a position from a position -> ../LineOfSightEvaluation
     * @memberof scoreGoalShot : uint8_t score for the Goal Shot opportunity for a position -> ../GoalShotEvaluation
     * @memberof scoreBlocking : uint8_t score for the potential to block from a position -> ../BlockingEvaluation
     */
    struct PositionScores {
        std::optional<double> scoreOpen;
        std::optional<double> scoreLineOfSight;
        std::optional<double> scoreGoalShot;
        std::optional<double> scoreBlocking;
    };

    /**
     * Combination of weights for each of the scores.
     * This will be used to determine the final score for a robot for a position.
     * All weights will be multiplied with the corresponding score and then normalized.
     *
     * @memberof weightOpen for scoreOpen
     * @memberof weightLineOfSight for scoreLineOfSight
     * @memberof weightGoalShot for scoreGoalShot
     */
    struct ScoreProfile{
        double weightOpen;
        double weightLineOfSight;
        double weightGoalShot;
        double weightBlocking;
    };

    /**
     * Structure with a position and its score
     * @memberof position Vector2 coordinates of a position
     * @memberof score The score for said position
     */
    struct ScoredPosition{
        Vector2 position;
        uint8_t score;
    };

    /**
     * Generalized Position Profiles to be used in plays.
     * They consist of a generalized weight combination.
     */
    constexpr ScoreProfile SafePosition = {1,1,0,0.5};
    constexpr ScoreProfile OffensivePosition = {1,0.5,0.5,0};
    constexpr ScoreProfile GoalShootPosition = {0, 0.5,1,0};
    constexpr ScoreProfile BlockingPosition = {0, 0.5,0,1};

    /**
     * Generalized Grids to be used in plays
     * These positions are meant to be used across plays to lower computations when computing which
     * play is best by reducing the amount of unique position that need to be evaluated.
     */
    //Scaling for B-Division
//    double scaleX = 0.75;
//    double scaleY = 1;

    inline static Grid gridRightTop = Grid(3*1.0, 3*1, 2.5*1.0, 2.2*1, 3, 3);
    inline static Grid gridRightMid = Grid(3*1.0, 0*1, 2.5*1.0, 3*1, 3, 3);
    inline static Grid gridRightBot = Grid(3*1.0, -3*1, 2.5*1.0, 2.2*1, 3, 3);
    inline static Grid gridMidFieldTop = Grid(-1*1.0, 3*1, 2*1.0, 2.2*1, 3, 3);
    inline static Grid gridMidFieldMid = Grid(-1*1.0, -1.5*1, 2*1.0, 3*1, 3, 3);
    inline static Grid gridMidFieldBot = Grid(-1*1.0, -3*1, 2*1.0, 2.2*1, 3, 3);
    inline static Grid gridLeftTop = Grid(-5*1.0, 3*1, 2.5*1.0, 2.2*1, 3, 3);
    inline static Grid gridLeftMid = Grid(-5*1.0, -1.5*1, 2.5*1.0, 3*1, 3, 3);
    inline static Grid gridLeftBot = Grid(-5*1.0, -3*1, 2.5*1.0, 2.2*1, 3, 3);


    /**
     * Generalized Keys for passing information form the old play to the new.
     * Usage in the storePlayInfo where KeyInfo is the key for the elements in the map.
     */
    enum class KeyInfo{
        isPasser = 0,   // Robot that passes the ball last play
        isReceiver,     // Robot that should receive the ball (as passer shot to there)
        isShooter,      // Robot that Shot the ball last play
        hasBall         // Robot that had the ball last play
    };

    /**
     * Generalized information structure for the map of storePlayInfo.
     * Allows for saving specific information from the old play to the new.
     */
    struct StoreInfo {
        std::optional<int> robotID;
        std::optional<Vector2> robotPosition;
        std::optional<Vector2> moveToPosition;
        std::optional<Vector2> defendPosition;
        std::optional<Vector2> shootAtPosition;
        std::optional<Vector2> passToRobot;
    };

    /**
     * Place to store info in that is needed between Plays. Used in storePlayInfo.
     */
    using PlayInfos = std::unordered_map<KeyInfo, StoreInfo>;
}
#endif //RTT_GENERALIZATIONCONSTANTS_H
