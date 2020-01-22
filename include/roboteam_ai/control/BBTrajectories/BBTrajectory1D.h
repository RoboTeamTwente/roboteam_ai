//
// Created by rolf on 15-01-20.
//

#ifndef RTT_BBTRAJECTORY1D_H
#define RTT_BBTRAJECTORY1D_H

/**
 * @author rolfvdhulst
 * @date 15 January 2020
 * @tparam num : float or double
 */
template<class num>
struct BBTrajectoryPart {
  num tEnd;
  num acc;
  num startVel;
  num startPos;
};

/**
 * @author rolfvdhulst
 * @date 15 January 2020
 * @tparam num : float or double
 */
template<class num>
struct PosVelAcc {
  explicit PosVelAcc(num pos, num vel, num acc)
          :
          pos{pos},
          vel{vel},
          acc{acc} { };

  num pos;
  num vel;
  num acc;
};

/**
 * @author rolfvdhulst
 * @date 15 January 2020
 * @tparam num : float or double
 */
template<class num>
class BBTrajectory1D {
    public:
        typedef BBTrajectoryPart<num> part;
        PosVelAcc<num> getValues(num t) const noexcept ;
        num getPosition(num t) const noexcept;
        num getVelocity(num t) const noexcept;
        num getAcceleration(num t) const noexcept;
        num getTotalTime() const noexcept ;
        bool inLastPart(num t) const noexcept;
        /**
        * Generate a time-optimal trajectory given the parameters
        * @param initialPos Current position
        * @param initialVel Current velocity
        * @param finalPos Final position
        * @param maxVel Maximum allowed velocity (absolute)
        * @param maxAcc maximum allowed acceleration/deceleration
        */
        void generateTrajectory(num initialPos, num initialVel, num finalPos, num maxVel, num maxAcc) noexcept ;
        BBTrajectory1D(num initialPos, num initialVel,num finalPos, num maxVel, num maxAcc) noexcept;
        BBTrajectory1D() noexcept = default;
    private:

        /**
         * Computes the position where we would end if we would initiate a maximal break given current state
         * @param pos Current position
         * @param vel Current velocity
         * @param maxAcc Maximum allowed acceleration/deceleration
         * @return final position (with zero velocity)
         */
        num fullBrakePos(num pos, num vel, num accMax) noexcept ;
        /**
         * Computes the position where we would end if we first accelerate to a target velocity and then immediately decelerate to 0
         * @param pos0 Current position
         * @param vel0 Current velocity
         * @param vel1 Target velocity
         * @param accMax Maximum allowed acceleration/deceleration
         * @return final position (with zero velocity)
         */
        num accelerateBrakePos(num pos0, num vel0, num vel1, num accMax) noexcept;
        /**
         * Generates a time-optimal triangular profile (accelerating and then breaking)
         * @param initialPos Current position
         * @param initialVel Current velocity
         * @param finalPos Desired final psition
         * @param maxAcc Maximum allowed acceleration/deceleration
         * @param invertedSign Specifies whether or not to invert the sign of velocity / acceleration. This is more of a techniality
         */
        void triangularProfile(num initialPos, num initialVel, num finalPos, num maxAcc, bool invertedSign) noexcept;
        /**
         * Generates a time-optimal trapezoidal profile (accelerating, coasting, then breaking)
         * @param initialPos Current position
         * @param initialVel Current velocity
         * @param maxVel Coasting velocity
         * @param finalPos Desired final position
         * @param maxAcc Maximum allowed acceleration/deceleration
         */
        void trapezoidalProfile(num initialPos, num initialVel, num finalPos, num maxVel, num maxAcc) noexcept;
        /**
         * Updates a part of the trajectory
         * @param index which part to update
         * @param tEnd time spent on trajectory segment
         * @param acc acceleration on segment
         * @param vel initial velocity of segment
         * @param pos initial position of segment
         */
        void updatePart(int index, num tEnd, num acc, num vel, num pos) noexcept;

        part parts[3];
        unsigned short int numParts{};
        //TODO: if in the future one of these fields is uncalled (particularly initialVel, intialPos are redundant) they can be removed.
        //The memory overhead for now seems minimal
        num m_initialPos; // m
        num m_finalPos; // m
        num m_initialVel; // m/s
        num m_maxAcc; // m/s^2
        num m_maxVel; // m/s

};
#include "src/control/BBTrajectories/BBTrajectory1D.tpp"
#endif //RTT_BBTRAJECTORY1D_H
